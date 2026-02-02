// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <rcl_action/default_qos.h>

#include <connect/action/actionClient.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace connect_plugins {
    class FibonacciActionClient final : public action::ActionClient {
    public:
        void init(Service_Action_t &definition, const rclcpp::Node::SharedPtr &node, const rclcpp::CallbackGroup::SharedPtr &callbackGroup, const std::string &remoteEndpoint, const std::function<void(const std::shared_ptr<const std::vector<uint8_t>> &)> send) override {
            // perform the base initialization
            ActionClient::init(definition, node, callbackGroup, remoteEndpoint, send);

            // define the expected size
            this->expectedSize = 4;

            // create the client
            if (definition.serviceQos == nullptr || definition.feedbackQos == nullptr || definition.statusQos == nullptr) {
                RCLCPP_ERROR(this->logger.get(), "Action with channel %d has no QoS definitions, falling back to default profile", definition.channel);
                this->options = rcl_action_client_get_default_options();
            } else {
                this->options = {
                    .goal_service_qos = *definition.serviceQos,
                    .result_service_qos = *definition.serviceQos,
                    .cancel_service_qos = *definition.serviceQos,
                    .feedback_topic_qos = *definition.feedbackQos,
                    .status_topic_qos = *definition.statusQos,
                    .allocator = rcl_get_default_allocator()
                };
            }

            this->client = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(node, "fibonacci", callbackGroup, this->options);

            switch (this->definition->compression->compressor) {
                case LZ4_DEFAULT:
                    RCLCPP_INFO(this->logger.get(), "Action Client with LZ4_DEFAULT (%d) compression for type %s on action-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                case LZ4_HC:
                    RCLCPP_INFO(this->logger.get(), "Action Client with LZ4_HC (%d) compression for type %s on action-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                case ZLIB:
                    RCLCPP_INFO(this->logger.get(), "Action Client with ZLIB (%d) compression for type %s on action-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                default:
                    RCLCPP_INFO(this->logger.get(), "Action Client without compression for type %s on action-channel %d created", this->definition->type.c_str(), this->definition->channel);
            }
        }

        void test(Service_Action_t &definition, const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &nodeBaseInterface, const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr &nodeGraphInterface, const rclcpp::node_interfaces::NodeClockInterface::SharedPtr &/*nodeClockInterface*/, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &nodeLoggingInterface, const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr &nodeWaitablesInterface, const rclcpp::CallbackGroup::SharedPtr &callbackGroup) const override {
            rcl_action_client_options_t options = {};

            if (definition.serviceQos == nullptr || definition.feedbackQos == nullptr || definition.statusQos == nullptr) {
                RCLCPP_ERROR(this->logger.get(), "Action with channel %d has no QoS definitions, falling back to default profile", definition.channel);
                options = rcl_action_client_get_default_options();
            } else {
                options = {
                    .goal_service_qos = *definition.serviceQos,
                    .result_service_qos = *definition.serviceQos,
                    .cancel_service_qos = *definition.serviceQos,
                    .feedback_topic_qos = *definition.feedbackQos,
                    .status_topic_qos = *definition.statusQos,
                    .allocator = rcl_get_default_allocator()
                };
            }

            rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr client = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(nodeBaseInterface, nodeGraphInterface, nodeLoggingInterface, nodeWaitablesInterface, "fibonacci", callbackGroup, options);
            client.reset();
        }

    protected:
        bool waitForAction() override {
            return this->client->wait_for_action_server(std::chrono::seconds(1));
        }

        void asyncCancelGoal(std::shared_ptr<ServiceActionHandle> handle) override {
            // if we have no goal handle we can not cancel anything
            if (handle->clientGoalHandle == nullptr) {
                RCLCPP_WARN(this->logger.get(), "Could not cancel goal because client goal handle is null");
                this->cancelRejected(handle);
                return;
            }

            // perform the local request
            this->client->async_cancel_goal(std::static_pointer_cast<rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>>(handle->clientGoalHandle), [this, handle](rclcpp_action::Client<example_interfaces::action::Fibonacci>::CancelResponse::SharedPtr cancelResponse){
                if (cancelResponse->return_code == cancelResponse->ERROR_NONE || cancelResponse->return_code == cancelResponse->ERROR_GOAL_TERMINATED) {
                    this->cancelAccepted(handle);
                } else {
                    this->cancelRejected(handle);
                }
            });
        }

        void asyncSendGoal(const std::unique_ptr<ServiceActionMessage> &message, std::shared_ptr<ServiceActionHandle> handle) override {
            const std::span<uint8_t> &requestData = message->getData();
            
            // construct the goal
            example_interfaces::action::Fibonacci::Goal goal = example_interfaces::action::Fibonacci::Goal();
            goal.order =
                    (static_cast<int32_t>(requestData[0])) |
                    (static_cast<int32_t>(requestData[1]) << 8) |
                    (static_cast<int32_t>(requestData[2]) << 16) |
                    (static_cast<int32_t>(requestData[3]) << 24);

            // construct the goal options
            rclcpp_action::Client<example_interfaces::action::Fibonacci>::SendGoalOptions goalOptions = rclcpp_action::Client<example_interfaces::action::Fibonacci>::SendGoalOptions();
            goalOptions.goal_response_callback = [this, handle](rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr goalHandle) {
                // if the goal was rejected, there will be no goal handle
                if (goalHandle == nullptr) {
                    this->goalRejected(handle);
                }

                // otherwise it was accepted or is already processing or was already aborted or cancelled
                this->goalAccepted(handle, goalHandle);
            };
            goalOptions.feedback_callback = [this, handle](rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr /*goalHandle*/, std::shared_ptr<const example_interfaces::action::Fibonacci::Feedback> feedback) {
                std::vector<uint8_t> data(4 * feedback->sequence.size());
                int i = 0;
                for (const int32_t &val: feedback->sequence) {
                    data[i * 4] = static_cast<uint8_t>((val) & 0xFF);
                    data[i * 4 + 1] = static_cast<uint8_t>((val >> 8) & 0xFF);
                    data[i * 4 + 2] = static_cast<uint8_t>((val >> 16) & 0xFF);
                    data[i * 4 + 3] = static_cast<uint8_t>((val >> 24) & 0xFF);
                    i++;
                }
                this->feedback(data, handle);
            };
            goalOptions.result_callback = [this, handle](rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::WrappedResult wrappedResult) {
                std::vector<uint8_t> data(4 * wrappedResult.result->sequence.size());
                int i = 0;
                for (const int32_t &val: wrappedResult.result->sequence) {
                    data[i * 4] = static_cast<uint8_t>((val) & 0xFF);
                    data[i * 4 + 1] = static_cast<uint8_t>((val >> 8) & 0xFF);
                    data[i * 4 + 2] = static_cast<uint8_t>((val >> 16) & 0xFF);
                    data[i * 4 + 3] = static_cast<uint8_t>((val >> 24) & 0xFF);
                    i++;
                }

                switch (wrappedResult.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        this->resultSucceeded(data, handle);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        this->resultCanceled(data, handle);
                        break;
                    default:
                        this->resultAborted(data, handle);
                        break;
                }
            };

            // send the goal
            this->client->async_send_goal(goal, goalOptions);
        }

    private:
        rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr client;

        rcl_action_client_options_t options = {};
    };
} // namespace connect_plugins

PLUGINLIB_EXPORT_CLASS(connect_plugins::FibonacciActionClient, action::ActionClient)
