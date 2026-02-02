// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <connect/service/serviceClient.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace connect_plugins {
class AddTwoIntsServiceClient final : public service::ServiceClient {
   public:
    void init(Service_Action_t &definition, const rclcpp::Node::SharedPtr &node, const rclcpp::CallbackGroup::SharedPtr &callbackGroup, const std::string &remoteEndpoint, const std::function<void(const std::shared_ptr<const std::vector<uint8_t>> &)> send) override {
        // perform the base initialization
        ServiceClient::init(definition, node, callbackGroup, remoteEndpoint, send);

        // define the error vector
        this->error = std::vector<uint8_t>{
            static_cast<uint8_t>(std::numeric_limits<int64_t>::min() & 0xFF),
            static_cast<uint8_t>((std::numeric_limits<int64_t>::min() >> 8) & 0xFF),
            static_cast<uint8_t>((std::numeric_limits<int64_t>::min() >> 16) & 0xFF),
            static_cast<uint8_t>((std::numeric_limits<int64_t>::min() >> 24) & 0xFF),
            static_cast<uint8_t>((std::numeric_limits<int64_t>::min() >> 32) & 0xFF),
            static_cast<uint8_t>((std::numeric_limits<int64_t>::min() >> 40) & 0xFF),
            static_cast<uint8_t>((std::numeric_limits<int64_t>::min() >> 48) & 0xFF),
            static_cast<uint8_t>((std::numeric_limits<int64_t>::min() >> 56) & 0xFF)};

        // define the expected size
        this->expectedSize = 16;

        // create the client
        if (this->definition->serviceQos == nullptr) {
            RCLCPP_ERROR(this->logger.get(), "Service with channel %d has no QoS definition, falling back to default profile", this->definition->channel);
            this->client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default), callbackGroup);
        } else {
            this->client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*this->definition->serviceQos), *this->definition->serviceQos), callbackGroup);
        }

        switch (this->definition->compression->compressor) {
            case LZ4_DEFAULT:
                RCLCPP_INFO(this->logger.get(), "Service Client with LZ4_DEFAULT (%d) compression for type %s on service-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                break;
            case LZ4_HC:
                RCLCPP_INFO(this->logger.get(), "Service Client with LZ4_HC (%d) compression for type %s on service-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                break;
            case ZLIB:
                RCLCPP_INFO(this->logger.get(), "Service Client with ZLIB (%d) compression for type %s on service-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                break;
            default:
                RCLCPP_INFO(this->logger.get(), "Service Client without compression for type %s on service-channel %d created", this->definition->type.c_str(), this->definition->channel);
        }
    }

    void test(Service_Action_t &definition, const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &nodeBaseInterface, const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr &nodeGraphInterface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr &nodeServicesInterface, const rclcpp::CallbackGroup::SharedPtr &callbackGroup) const override {
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;

        if (definition.serviceQos == nullptr) {
            client = rclcpp::create_client<example_interfaces::srv::AddTwoInts>(nodeBaseInterface, nodeGraphInterface, nodeServicesInterface, "add_two_ints", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default), callbackGroup);
        } else {
            client = rclcpp::create_client<example_interfaces::srv::AddTwoInts>(nodeBaseInterface, nodeGraphInterface, nodeServicesInterface, "add_two_ints", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*definition.serviceQos), *definition.serviceQos), callbackGroup);
        }

        client.reset();
    }

    void cancel(const int64_t requestId) override {
        this->client->remove_pending_request(requestId);
    }

    void cancelAll() override {
        // this will always be called during shutdown, even then when node->create_client has failed
        if (this->client)
            this->client->prune_pending_requests();
    }

    bool waitForService() override {
        return this->client->wait_for_service(std::chrono::seconds(1));
    }

    int64_t asyncSendRequest(const std::unique_ptr<ServiceActionMessage> &message, std::shared_ptr<ServiceActionHandle> handle) override {
        // deserialize the request
        const std::span<uint8_t> &requestData = message->getData();
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

        request->a =
            (static_cast<int64_t>(requestData[0])) |
            (static_cast<int64_t>(requestData[1]) << 8) |
            (static_cast<int64_t>(requestData[2]) << 16) |
            (static_cast<int64_t>(requestData[3]) << 24) |
            (static_cast<int64_t>(requestData[4]) << 32) |
            (static_cast<int64_t>(requestData[5]) << 40) |
            (static_cast<int64_t>(requestData[6]) << 48) |
            (static_cast<int64_t>(requestData[7]) << 56);
        request->b =
            (static_cast<int64_t>(requestData[8])) |
            (static_cast<int64_t>(requestData[9]) << 8) |
            (static_cast<int64_t>(requestData[10]) << 16) |
            (static_cast<int64_t>(requestData[11]) << 24) |
            (static_cast<int64_t>(requestData[12]) << 32) |
            (static_cast<int64_t>(requestData[13]) << 40) |
            (static_cast<int64_t>(requestData[14]) << 48) |
            (static_cast<int64_t>(requestData[15]) << 56);

        // perform the local request
        const rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureAndRequestId future = this->client->async_send_request(request, [this, handle](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
            this->callback(future, handle);
        });
        return future.request_id;
    }

   protected:
    void callback(const rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture &future, const std::shared_ptr<ServiceActionHandle> &handle) {
        try {
            // get the response
            const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> &response = future.get();

            // serialize the response
            std::vector<uint8_t> data = std::vector<uint8_t>(8);

            data[0] = static_cast<uint8_t>(response->sum & 0xFF);
            data[1] = static_cast<uint8_t>((response->sum >> 8) & 0xFF);
            data[2] = static_cast<uint8_t>((response->sum >> 16) & 0xFF);
            data[3] = static_cast<uint8_t>((response->sum >> 24) & 0xFF);
            data[4] = static_cast<uint8_t>((response->sum >> 32) & 0xFF);
            data[5] = static_cast<uint8_t>((response->sum >> 40) & 0xFF);
            data[6] = static_cast<uint8_t>((response->sum >> 48) & 0xFF);
            data[7] = static_cast<uint8_t>((response->sum >> 56) & 0xFF);

            // send and done
            this->sendAndDone(data, handle);
        } catch (std::exception &e) {
            // send and done
            RCLCPP_ERROR(this->logger.get(), "Local service call to service of type %s failed with error: %s", this->definition->type.c_str(), e.what());
            this->sendAndDone(this->error, handle);
        }
    }

   private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;
};
}  // namespace connect_plugins

PLUGINLIB_EXPORT_CLASS(connect_plugins::AddTwoIntsServiceClient, service::ServiceClient)
