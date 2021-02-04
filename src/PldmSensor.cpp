/*
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "PldmSensor.hpp"

#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <libpldm/base.h>
#include <libpldm/platform.h>
#include <libpldm/pldm.h>
#include <math.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include "ncsi_util.hpp"

extern ncsi_data_len;
extern  ncsi_data;

constexpr const bool debug = false;

constexpr const char* configInterface =
    "xyz.openbmc_project.Configuration.PldmSensor";
static constexpr double ipmbMaxReading = 0xFF;
static constexpr double ipmbMinReading = 0;

static constexpr uint8_t meAddress = 1;
static constexpr uint8_t lun = 0;
static constexpr uint8_t hostSMbusIndexDefault = 0x03;

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

#define NCSI_MAX_PAYLOAD 1480 /* maximum payload size*/
/* max ethernet frame size = 1518 */
/* ethernet headr (14) + nc-si header (16) + nc-si payload (1480) + nc-si
 * checksum (4) + 4 (FCS) = 1518*/

#define NCSI_PLDM_REQUEST 0x51
#define PLDM_COMMON_REQ_LEN 3 // 3 bytes common field for PLDM requests
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
#define MAX_PLDM_MSG_SIZE (NCSI_MAX_PAYLOAD)
typedef struct
{
    uint16_t payload_size;
    unsigned char common[PLDM_COMMON_REQ_LEN];
    unsigned char payload[MAX_PLDM_MSG_SIZE - PLDM_COMMON_REQ_LEN];
} __attribute__((packed)) pldm_cmd_req;
// cdb for pldm cmd CMD_GET_SENSOR_READING
typedef struct
{
    uint16_t sensorId;
    uint8_t rearmEventState;
} __attribute__((packed)) PLDM_Get_Sensor_Reading_t;

uint16_t sensorID;
std::string sdrSensorType;
uint8_t reqIID;
uint8_t pldmType;
uint8_t pldmSensorCmd;
uint8_t mctp_eid;

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

boost::container::flat_map<std::string, std::unique_ptr<PldmSensor>> sensors;

std::unique_ptr<boost::asio::deadline_timer> initCmdTimer;

PldmSensor::PldmSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                       boost::asio::io_service& io,
                       const std::string& sensorName,
                       const std::string& sensorConfiguration,
                       sdbusplus::asio::object_server& objectServer,
                       std::vector<thresholds::Threshold>&& thresholdData,
                       uint8_t deviceAddress, uint8_t hostSMbusIndex,
                       std::string& sensorTypeName) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdData), sensorConfiguration,
           "xyz.openbmc_project.Configuration.ExitAirTemp", ipmbMaxReading,
           ipmbMinReading, conn, PowerState::on),
    deviceAddress(deviceAddress), hostSMbusIndex(hostSMbusIndex),
    objectServer(objectServer), waitTimer(io)
{
    std::string dbusPath = sensorPathPrefix + sensorTypeName + "/" + name;

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface(dbusPath, association::interface);
}

PldmSensor::~PldmSensor()
{
    waitTimer.cancel();
    objectServer.remove_interface(thresholdInterfaceWarning);
    objectServer.remove_interface(thresholdInterfaceCritical);
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void PldmSensor::init(void)
{

    printf("Init \n");
    std::cout.flush();
    loadDefaults();
    setInitialProperties(dbusConnection);
#if 0
    if (initCommand)
    {
        runInitCmd();
    }
#endif
    read();
}


void PldmSensor::runInitCmd()
{
    if (initCommand)
    {
        dbusConnection->async_method_call(
            [this](boost::system::error_code ec,
                   const IpmbMethodType& response) {
                const int& status = std::get<0>(response);

                if (ec || status)
                {
                    std::cerr
                        << "Error setting init command for device: " << name
                        << "\n";
                }
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, *initCommand, initData);
    }
}

void PldmSensor::loadDefaults()
{
    // if (sensorType == PldmType::pldmSensor)
    if (sdrSensorType == "PldmSensor")
    {
        reqIID = 0x81;
        pldmType = 2;
        pldmSensorCmd = 0x11;
    }
    else
    {
        throw std::runtime_error("Invalid sensor type");
    }

    if (subType == IpmbSubType::util)
    {
        // Utilization need to be scaled to percent
        maxValue = 100;
        minValue = 0;
    }
}

void PldmSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

void PldmSensor::read(void)
{
    printf("inside Read-1 \n");
    std::cout.flush();
    uint16_t sensorId = 0x0001;
    bool8_t rearmEventState = 0x01;

    std::vector<uint8_t> requestMsg(sizeof(pldm_msg_hdr) +
                                    PLDM_GET_SENSOR_READING_REQ_BYTES);

    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    auto rc =
        encode_get_sensor_reading_req(0, sensorId, rearmEventState, request);

    printf("Data Encode %X %X %X %X \n", request->hdr.command,
           request->payload[0], request->payload[1], request->payload[2]);
    std::cout.flush();
    /** @brief MCTP EID of host firmware */
    constexpr uint8_t PLDM_ENTITY_ID = 8;
    const uint8_t MCTP_MSG_TYPE_PLDM = 1;
    std::vector<uint8_t> responseMsg;
    // Insert the PLDM message type and EID at the beginning of the
    // msg.
    requestMsg.insert(requestMsg.begin(), MCTP_MSG_TYPE_PLDM);
    requestMsg.insert(requestMsg.begin(), mctp_eid);

         int package = 0;
    int channel = 0;
    int ifindex = 2;
    int opcode  = 81;
    short payload_length = 12;
    uint8_t* payload = &requestMsg[0];

    sendCommand(ifindex, package, channel, opcode, payload_length, payload);

    printf("NCSI Response Payload length = %d\n", ncsi_data_len);
    printf("Response Payload:\n");
    for (int i = 0; i < ncsi_data_len; ++i) {
        printf("0x%02x ", *(ncsi_data+i));
    }
    printf("\n");


    /*if (mctp_eid != PLDM_ENTITY_ID)
    {
        int fd = pldm_open();
        if (-1 == fd)
        {
            std::cerr << "failed to init mctp "
                      << "\n";
            // return -1;
        }
        uint8_t* responseMessage = nullptr;
        size_t responseMessageSize{};
    printf("trace-1 \n");
    std::cout.flush();
        pldm_send_recv(mctp_eid, fd, requestMsg.data() + 2,
                       requestMsg.size() - 2, &responseMessage,
                       &responseMessageSize);
    printf("trace-2 \n");
    std::cout.flush();
        responseMsg.resize(responseMessageSize);
        memcpy(responseMsg.data(), responseMessage, responseMsg.size());

        free(responseMessage);
    }
    else
    {
        mctpSockSendRecv(requestMsg, responseMsg, mctpVerbose);
        Logger(pldmVerbose, "Response Message:", "");
        printBuffer(responseMsg, pldmVerbose);
        responseMsg.erase(responseMsg.begin(),
                          responseMsg.begin() + 2 ;
    }

    auto responsePtr = reinterpret_cast<struct pldm_msg*>(responseMsg.data());
    printf("Read-2 \n");
    std::cout.flush();
    constexpr auto hdrSize = sizeof(pldm_msg_hdr);
    std::array<uint8_t, hdrSize + PLDM_GET_SENSOR_READING_MIN_RESP_BYTES + 3>
        responseMsg1{};

    uint8_t completionCode = 0;
    uint8_t retcompletionCode;
    uint8_t retsensor_dataSize = PLDM_SENSOR_DATA_SIZE_UINT32;
    uint8_t retsensor_operationalState;
    uint8_t retsensor_event_messageEnable;
    uint8_t retpresentState;
    uint8_t retpreviousState;
    uint8_t reteventState;
    uint8_t retpresentReading[4];

    auto rcDec = decode_get_sensor_reading_resp(
        responsePtr, responseMsg1.size() - hdrSize, &retcompletionCode,
        &retsensor_dataSize, &retsensor_operationalState,
        &retsensor_event_messageEnable, &retpresentState, &retpreviousState,
        &reteventState, retpresentReading);

    if (rcDec != PLDM_SUCCESS || completionCode != PLDM_SUCCESS)
    {
        std::cerr << "Response Message Error: "
                  << "rc=" << rc << ",cc=" << (int)completionCode << "\n";
        return;
    }

    updateValue(retpresentReading[0]);
    printf("Read-3 \n");
    std::cout.flush(); */
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<PldmSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        std::cerr << "Connection not created\n";
        return;
    }
    dbusConnection->async_method_call(
        [&](boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                std::cerr << "Error contacting entity manager\n";
                return;
            }
            for (const auto& pathPair : resp)
            {
                for (const auto& entry : pathPair.second)
                {
                    if (entry.first != configInterface)
                    {
                        continue;
                    }

                    std::string name =
                        loadVariant<std::string>(entry.second, "Name");

                    std::vector<thresholds::Threshold> sensorThresholds;
                    if (!parseThresholdsFromConfig(pathPair.second,
                                                   sensorThresholds))
                    {
                        std::cerr << "error populating thresholds for " << name
                                  << "\n";
                    }
                    uint8_t reqAddress =
                        loadVariant<uint8_t>(entry.second, "Address");

                    std::string readType =
                        loadVariant<std::string>(entry.second, "Type");
                    if (readType == "PldmSensor")
                    {
                        sensorID = loadVariant<std::uint16_t>(entry.second,
                                                              "SensorID");
                        mctp_eid = loadVariant<std::uint8_t>(entry.second,
                                                              "MctpID");
                        sdrSensorType = readType;
                        std::cerr << "Sens Type " << readType << "\n";
                        std::cerr << "Pldm " << sdrSensorType << "\n";
                        std::cerr << "mctp_eid " << mctp_eid << "\n";
                        std::cout.flush();
                    }

                    std::cerr << "Sensor Type " << sdrSensorType << "\n";
                    printf("Trace-4 \n");
                    std::cout.flush();
                    uint8_t hostSMbusIndex = hostSMbusIndexDefault;
                    auto findSmType = entry.second.find("HostSMbusIndex");
                    if (findSmType != entry.second.end())
                    {
                        hostSMbusIndex = std::visit(
                            VariantToUnsignedIntVisitor(), findSmType->second);
                    }

                    /* Default sensor type is "temperature" */
                    std::string sensorTypeName = "temperature";
                    auto findType = entry.second.find("SensorType");
                    if (findType != entry.second.end())
                    {
                        sensorTypeName = std::visit(VariantToStringVisitor(),
                                                    findType->second);
                    }

                    printf("Trace-5 \n");
                    std::cout.flush();
                    auto& sensor = sensors[name];
                    sensor = std::make_unique<PldmSensor>(
                        dbusConnection, io, name, pathPair.first, objectServer,
                        std::move(sensorThresholds), reqAddress, hostSMbusIndex,
                        sensorTypeName);

                    printf("Trace-6 \n");
                    std::cout.flush();

                    sensor->init();
                }
            }
        },
        entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");
}

void reinitSensors(sdbusplus::message::message& message)
{
    constexpr const size_t reinitWaitSeconds = 2;
    std::string objectName;
    boost::container::flat_map<std::string, std::variant<std::string>> values;
    message.read(objectName, values);

    auto findStatus = values.find(power::property);
    if (findStatus != values.end())
    {
        bool powerStatus = boost::ends_with(
            std::get<std::string>(findStatus->second), "Running");
        if (powerStatus)
        {
            if (!initCmdTimer)
            {
                // this should be impossible
                return;
            }
            // we seem to send this command too fast sometimes, wait before
            // sending
            initCmdTimer->expires_from_now(
                boost::posix_time::seconds(reinitWaitSeconds));

            initCmdTimer->async_wait([](const boost::system::error_code ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }

                for (const auto& sensor : sensors)
                {
                    if (sensor.second)
                    {
                        sensor.second->runInitCmd();
                    }
                }
            });
        }
    }
}

int main()
{

    printf("Inside Main \n");
    std::cout.flush();
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.PldmSensor");
    sdbusplus::asio::object_server objectServer(systemBus);

    initCmdTimer = std::make_unique<boost::asio::deadline_timer>(io);

    io.post([&]() { createSensors(io, objectServer, sensors, systemBus); });

    boost::asio::deadline_timer configTimer(io);

    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message&) {
            configTimer.expires_from_now(boost::posix_time::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                createSensors(io, objectServer, sensors, systemBus);
                if (sensors.empty())
                {
                    std::cout << "Configuration not detected\n";
                }
            });
        };

    sdbusplus::bus::match::match configMatch(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + configInterface +
            "'",
        eventHandler);

    sdbusplus::bus::match::match powerChangeMatch(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + "',arg0='" +
            std::string(power::interface) + "'",
        reinitSensors);

    io.run();
}
