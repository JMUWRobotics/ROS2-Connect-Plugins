// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <connect/service/serviceServer.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace connect_plugins {
    class AddTwoIntsServiceServer final : public service::ServiceServer {
    public:
        void init(Service_Action_t &definition, const rclcpp::Node::SharedPtr &node, const rclcpp::CallbackGroup::SharedPtr &callbackGroup, const std::string &remoteEndpoint, const std::function<void(const std::shared_ptr<const std::vector<uint8_t>> &)> send) override {
            // perform the base initialization
            ServiceServer::init(definition, node, callbackGroup, remoteEndpoint, send);

            // create the server
            if (definition.serviceQos == nullptr) {
                RCLCPP_ERROR(this->logger.get(), "Service with channel %d has no QoS definition, falling back to default profile", definition.channel);
                this->server = node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&AddTwoIntsServiceServer::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default), callbackGroup);
            } else {
                this->server = node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&AddTwoIntsServiceServer::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*definition.serviceQos), *definition.serviceQos), callbackGroup);
            }

            switch (this->definition->compression->compressor) {
                case LZ4_DEFAULT:
                    RCLCPP_INFO(this->logger.get(), "Service Server with LZ4_DEFAULT (%d) compression for type %s on service-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                case LZ4_HC:
                    RCLCPP_INFO(this->logger.get(), "Service Server with LZ4_HC (%d) compression for type %s on service-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                case ZLIB:
                    RCLCPP_INFO(this->logger.get(), "Service Server with ZLIB (%d) compression for type %s on service-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                default:
                    RCLCPP_INFO(this->logger.get(), "Service Server without compression for type %s on service-channel %d created", this->definition->type.c_str(), this->definition->channel);
            }
        }

        void test(Service_Action_t &definition, const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &nodeBaseInterface, const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr & /*nodeGraphInterface*/, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr &nodeServicesInterface, const rclcpp::CallbackGroup::SharedPtr &callbackGroup) const override {
            rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server;

            if (definition.serviceQos == nullptr) {
                server = rclcpp::create_service<example_interfaces::srv::AddTwoInts>(nodeBaseInterface, nodeServicesInterface, "add_two_ints", [](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> &/*request*/, const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> &/*response*/) {
                }, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default), callbackGroup);
            } else {
                server = rclcpp::create_service<example_interfaces::srv::AddTwoInts>(nodeBaseInterface, nodeServicesInterface, "add_two_ints", [](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> &/*request*/, const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> &/*response*/) {
                }, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*definition.serviceQos), *definition.serviceQos), callbackGroup);
            }

            server.reset();
        }

        void cancel(const std::shared_ptr<ServiceActionHandle> &handle) override {
            example_interfaces::srv::AddTwoInts::Response response;
            response.sum = this->error;
            rmw_request_id_t request = handle->requestHeader();
            this->server->send_response(request, response);
        }

    private:
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server;

        const int64_t error = std::numeric_limits<int64_t>::min();

        void callback(const std::shared_ptr<rmw_request_id_t> &requestHeader, const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> &request, const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> &response) {
            // serialize the request
            std::vector<uint8_t> data = std::vector<uint8_t>(16);

            data[0] = static_cast<uint8_t>(request->a & 0xFF);
            data[1] = static_cast<uint8_t>((request->a >> 8) & 0xFF);
            data[2] = static_cast<uint8_t>((request->a >> 16) & 0xFF);
            data[3] = static_cast<uint8_t>((request->a >> 24) & 0xFF);
            data[4] = static_cast<uint8_t>((request->a >> 32) & 0xFF);
            data[5] = static_cast<uint8_t>((request->a >> 40) & 0xFF);
            data[6] = static_cast<uint8_t>((request->a >> 48) & 0xFF);
            data[7] = static_cast<uint8_t>((request->a >> 56) & 0xFF);

            data[8] = static_cast<uint8_t>(request->b & 0xFF);
            data[9] = static_cast<uint8_t>((request->b >> 8) & 0xFF);
            data[10] = static_cast<uint8_t>((request->b >> 16) & 0xFF);
            data[11] = static_cast<uint8_t>((request->b >> 24) & 0xFF);
            data[12] = static_cast<uint8_t>((request->b >> 32) & 0xFF);
            data[13] = static_cast<uint8_t>((request->b >> 40) & 0xFF);
            data[14] = static_cast<uint8_t>((request->b >> 48) & 0xFF);
            data[15] = static_cast<uint8_t>((request->b >> 56) & 0xFF);

            // now send the request to the "other side" and wait for a response
            const std::shared_ptr<ServiceActionHandle> handle = this->sendAndWait(data, requestHeader, 8);
            if (!rclcpp::ok()) {
                return;
            }
            else if (handle == nullptr) {
                response->sum = this->error;
                return;
            }

            // deserialize the response
            // we do not need to acquire a lock since at this point, handle->handlingGoal will be false, therefore the goalResponse is protected
            const std::span<uint8_t> &responseData = handle->goalResponse->getData();
            response->sum =
                    (static_cast<int64_t>(responseData[0])) |
                    (static_cast<int64_t>(responseData[1]) << 8) |
                    (static_cast<int64_t>(responseData[2]) << 16) |
                    (static_cast<int64_t>(responseData[3]) << 24) |
                    (static_cast<int64_t>(responseData[4]) << 32) |
                    (static_cast<int64_t>(responseData[5]) << 40) |
                    (static_cast<int64_t>(responseData[6]) << 48) |
                    (static_cast<int64_t>(responseData[7]) << 56);
            
            // done
            this->done(handle);
        }
    };
} // namespace connect_plugins

PLUGINLIB_EXPORT_CLASS(connect_plugins::AddTwoIntsServiceServer, service::ServiceServer)
