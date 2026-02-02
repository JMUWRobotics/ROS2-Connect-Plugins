// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <rcl_action/default_qos.h>

#include <connect/action/actionServer.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace connect_plugins {
    class FibonacciActionServer final : public action::ActionServer {
    public:
        void init(Service_Action_t &definition, const rclcpp::Node::SharedPtr &node, const rclcpp::CallbackGroup::SharedPtr &callbackGroup, const std::string &remoteEndpoint, const std::function<void(const std::shared_ptr<const std::vector<uint8_t>> &)> send) override {
            // perform the base initialization
            ActionServer::init(definition, node, callbackGroup, remoteEndpoint, send);

            // create the server
            if (definition.serviceQos == nullptr || definition.feedbackQos == nullptr || definition.statusQos == nullptr) {
                RCLCPP_ERROR(this->logger.get(), "Action with channel %d has no QoS definitions, falling back to default profile", definition.channel);
                this->options = {
                    .goal_service_qos = rmw_qos_profile_services_default,
                    .cancel_service_qos = rmw_qos_profile_services_default,
                    .result_service_qos = rmw_qos_profile_services_default,
                    .feedback_topic_qos = rmw_qos_profile_default,
                    .status_topic_qos = rcl_action_qos_profile_status_default,
                    .allocator = rcl_get_default_allocator(),
                    .result_timeout = rcl_duration_t{RCUTILS_S_TO_NS(definition.resultTimeout)}
                };
            } else {
                this->options = {
                    .goal_service_qos = *definition.serviceQos,
                    .cancel_service_qos = *definition.serviceQos,
                    .result_service_qos = *definition.serviceQos,
                    .feedback_topic_qos = *definition.feedbackQos,
                    .status_topic_qos = *definition.statusQos,
                    .allocator = rcl_get_default_allocator(),
                    .result_timeout = rcl_duration_t{RCUTILS_S_TO_NS(definition.resultTimeout)}
                };
            }

            this->server = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
                node,
                "fibonacci",
                [this](const rclcpp_action::GoalUUID &gid, std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal) -> rclcpp_action::GoalResponse {
                    // serialize the goal
                    std::vector<uint8_t> data = std::vector<uint8_t>(4);

                    data[0] = static_cast<uint8_t>(goal->order & 0xFF);
                    data[1] = static_cast<uint8_t>((goal->order >> 8) & 0xFF);
                    data[1] = static_cast<uint8_t>((goal->order >> 16) & 0xFF);
                    data[1] = static_cast<uint8_t>((goal->order >> 24) & 0xFF);

                    // send and wait for the response
                    return this->sendGoalAndWait(gid, data);
                },
                [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> > goalHandle) -> rclcpp_action::CancelResponse {
                    return this->sendCancelAndWait(goalHandle->get_goal_id());
                },
                [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> > goalHandle) {
                    if (!this->handleAccepted(goalHandle, goalHandle->get_goal_id())) {
                        goalHandle->abort(std::make_shared<example_interfaces::action::Fibonacci::Result>());
                    }
                },
                this->options,
                callbackGroup
            );

            switch (this->definition->compression->compressor) {
                case LZ4_DEFAULT:
                    RCLCPP_INFO(this->logger.get(), "Action Server with LZ4_DEFAULT (%d) compression for type %s on action-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                case LZ4_HC:
                    RCLCPP_INFO(this->logger.get(), "Action Server with LZ4_HC (%d) compression for type %s on action-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                case ZLIB:
                    RCLCPP_INFO(this->logger.get(), "Action Server with ZLIB (%d) compression for type %s on action-channel %d created", this->definition->compression->rate, this->definition->type.c_str(), this->definition->channel);
                    break;
                default:
                    RCLCPP_INFO(this->logger.get(), "Action Server without compression for type %s on action-channel %d created", this->definition->type.c_str(), this->definition->channel);
            }
        }

        void test(Service_Action_t &definition, const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &nodeBaseInterface, const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr &/*nodeGraphInterface*/, const rclcpp::node_interfaces::NodeClockInterface::SharedPtr &nodeClockInterface, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &nodeLoggingInterface, const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr &nodeWaitablesInterface, const rclcpp::CallbackGroup::SharedPtr &callbackGroup) const override {
            rcl_action_server_options_t options = {};

            if (definition.serviceQos == nullptr || definition.feedbackQos == nullptr || definition.statusQos == nullptr) {
                RCLCPP_ERROR(this->logger.get(), "Action with channel %d has no QoS definitions, falling back to default profile", definition.channel);
                options = {
                    .goal_service_qos = rmw_qos_profile_services_default,
                    .cancel_service_qos = rmw_qos_profile_services_default,
                    .result_service_qos = rmw_qos_profile_services_default,
                    .feedback_topic_qos = rmw_qos_profile_default,
                    .status_topic_qos = rcl_action_qos_profile_status_default,
                    .allocator = rcl_get_default_allocator(),
                    .result_timeout = rcl_duration_t{RCUTILS_S_TO_NS(definition.resultTimeout)}
                };
            } else {
                options = {
                    .goal_service_qos = *definition.serviceQos,
                    .cancel_service_qos = *definition.serviceQos,
                    .result_service_qos = *definition.serviceQos,
                    .feedback_topic_qos = *definition.feedbackQos,
                    .status_topic_qos = *definition.statusQos,
                    .allocator = rcl_get_default_allocator(),
                    .result_timeout = rcl_duration_t{RCUTILS_S_TO_NS(definition.resultTimeout)}
                };
            }

            rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr server = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
                nodeBaseInterface,
                nodeClockInterface,
                nodeLoggingInterface,
                nodeWaitablesInterface,
                "fibonacci",
                [](const rclcpp_action::GoalUUID &, std::shared_ptr<const example_interfaces::action::Fibonacci::Goal>) -> rclcpp_action::GoalResponse {
                    return rclcpp_action::GoalResponse::REJECT;
                },
                [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> >) -> rclcpp_action::CancelResponse {
                    return rclcpp_action::CancelResponse::REJECT;
                },
                [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> >) {
                },
                options,
                callbackGroup);
            server.reset();
        }

        void abort(const std::shared_ptr<void> &goalHandle, const std::span<uint8_t> &data) override {
            const std::shared_ptr<example_interfaces::action::Fibonacci::Result> result = std::make_shared<example_interfaces::action::Fibonacci::Result>();
            if (data.size() % 4 != 0) {
                RCLCPP_WARN(this->logger.get(), "Received result of unexpected size, expected mod 4, got %lu, ignoring result", data.size());
            } else if (data.size() > 0) {
                for (size_t i = 0; i < data.size() - 4; i += 4) {
                    result->sequence.push_back(
                        (static_cast<int32_t>(data[i])) |
                        (static_cast<int32_t>(data[i + 1]) << 8) |
                        (static_cast<int32_t>(data[i + 2]) << 16) |
                        (static_cast<int32_t>(data[i + 3]) << 24));
                }
            }
            
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> > goal = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> >(goalHandle);
            goal->abort(result);
        }

        void cancel(const std::shared_ptr<void> &goalHandle, const std::span<uint8_t> &data) override {
            const std::shared_ptr<example_interfaces::action::Fibonacci::Result> result = std::make_shared<example_interfaces::action::Fibonacci::Result>();
            if (data.size() % 4 != 0) {
                RCLCPP_WARN(this->logger.get(), "Received result of unexpected size, expected mod 4, got %lu, ignoring result", data.size());
            } else if (data.size() > 0) {
                for (size_t i = 0; i < data.size() - 4; i += 4) {
                    result->sequence.push_back(
                        (static_cast<int32_t>(data[i])) |
                        (static_cast<int32_t>(data[i + 1]) << 8) |
                        (static_cast<int32_t>(data[i + 2]) << 16) |
                        (static_cast<int32_t>(data[i + 3]) << 24));
                }
            }

            const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> > goal = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> >(goalHandle);
            goal->canceled(result);
        }

        void succeed(const std::shared_ptr<void> &goalHandle, const std::span<uint8_t> &data) override {
            const std::shared_ptr<example_interfaces::action::Fibonacci::Result> result = std::make_shared<example_interfaces::action::Fibonacci::Result>();
            if (data.size() % 4 != 0) {
                RCLCPP_WARN(this->logger.get(), "Received result of unexpected size, expected mod 4, got %lu, ignoring result", data.size());
            } else if (data.size() > 0) {
                for (size_t i = 0; i < data.size() - 4; i += 4) {
                    result->sequence.push_back(
                        (static_cast<int32_t>(data[i])) |
                        (static_cast<int32_t>(data[i + 1]) << 8) |
                        (static_cast<int32_t>(data[i + 2]) << 16) |
                        (static_cast<int32_t>(data[i + 3]) << 24));
                }
            }
            
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> > goal = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> >(goalHandle);
            goal->succeed(result);
        }

        void feedback(const std::shared_ptr<void> &goalHandle, const std::span<uint8_t> &data) override {
            if (data.size() % 4 != 0) {
                RCLCPP_WARN(this->logger.get(), "Received feedback of unexpected size, expected mod 4, got %lu, dropping it", data.size());
            } else if (data.size() > 0) {
                const std::shared_ptr<example_interfaces::action::Fibonacci::Feedback> feedback = std::make_shared<example_interfaces::action::Fibonacci::Feedback>();
                for (size_t i = 0; i < data.size() - 4; i += 4) {
                    feedback->sequence.push_back(
                        (static_cast<int32_t>(data[i])) |
                        (static_cast<int32_t>(data[i + 1]) << 8) |
                        (static_cast<int32_t>(data[i + 2]) << 16) |
                        (static_cast<int32_t>(data[i + 3]) << 24));
                }
                
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> > goal = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> >(goalHandle);
                goal->publish_feedback(feedback);     
            }  
        }

    private:
        rcl_action_server_options_t options = {};

        rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr server;
    };
} // namespace connect_plugins

PLUGINLIB_EXPORT_CLASS(connect_plugins::FibonacciActionServer, action::ActionServer)
