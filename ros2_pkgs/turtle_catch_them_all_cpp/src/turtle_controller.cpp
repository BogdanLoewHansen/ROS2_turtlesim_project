#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        this->declare_parameter("catch_closest_turtle_fist", true);
        this->catch_closest_turtle_fist_ = this->get_parameter("catch_closest_turtle_fist").as_bool();

        this->turtlesim_up_ = false;

        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        this->alive_turtles_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles", 10, std::bind(&TurtleControllerNode::callbackAliveTurtles, this, std::placeholders::_1));
        this->pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControllerNode::callbackTurtlePose, this, std::placeholders::_1));
        this->control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&TurtleControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Turtle controller has been started.");
    }

private:
    double getDistanceFromCurrentPose(my_robot_interfaces::msg::Turtle turtle)
    {
        double dist_x = turtle.x - this->pose_.x;
        double dist_y = turtle.y - this->pose_.y;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void callbackAliveTurtles(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        if ((int)msg->turtles.size() > 0)
        {
            if (this->catch_closest_turtle_fist_)
            {
                my_robot_interfaces::msg::Turtle closest_turtle = msg->turtles.at(0);
                double closest_turtle_distance = getDistanceFromCurrentPose(closest_turtle);

                for (int i = 1; i < (int)msg->turtles.size(); i++)
                {
                    double distance = getDistanceFromCurrentPose(msg->turtles.at(i));
                    if (distance < closest_turtle_distance)
                    {
                        closest_turtle = msg->turtles.at(i);
                        closest_turtle_distance = distance;
                    }
                }

                this->turtle_to_catch_ = closest_turtle;
            }
            else
            {
                this->turtle_to_catch_ = msg->turtles.at(0);
            }
        }
    }

    void callbackTurtlePose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        this->pose_ = *msg;
        this->turtlesim_up_ = true;
    }

    void controlLoop()
    {

        if (!this->turtlesim_up_ || this->turtle_to_catch_.name == "")
        {
            return;
        }

        double dist_x = this->turtle_to_catch_.x - this->pose_.x;
        double dist_y = this->turtle_to_catch_.y - this->pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto newTwistMsg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            newTwistMsg.linear.x = 2 * distance;

            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - this->pose_.theta;

            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            newTwistMsg.angular.z = 6 * angle_diff;
        }
        else
        {
            newTwistMsg.linear.x = 0.0;
            newTwistMsg.angular.z = 0.0;

            this->catch_turtle_threads_.push_back(std::make_shared<std::thread>(
                std::bind(&TurtleControllerNode::callCatchTurtleService, this, this->turtle_to_catch_.name)));
            this->turtle_to_catch_.name = "";
        }

        this->cmd_vel_publisher_->publish(newTwistMsg);
    }

    void callCatchTurtleService(std::string turtle_name)
    {
        auto client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if (!(response->success))
            {
                RCLCPP_INFO(this->get_logger(), "Turtle %s could not be caught.", turtle_name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    bool turtlesim_up_;
    bool catch_closest_turtle_fist_;
    my_robot_interfaces::msg::Turtle turtle_to_catch_;
    turtlesim::msg::Pose pose_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    std::vector<std::shared_ptr<std::thread>> catch_turtle_threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}