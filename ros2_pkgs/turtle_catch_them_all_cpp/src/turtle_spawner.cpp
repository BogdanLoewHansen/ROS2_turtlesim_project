#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner")
    {
        this->declare_parameter("spawn_frequency", 1.0);
        this->declare_parameter("turtle_name_prefix", "turtle");

        this->spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();
        this->turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();

        this->turtle_counter_ = 1;

        this->alive_turtles_publihser_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10);
        this->catch_turtle_service_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "catch_turtle", std::bind(&TurtleSpawnerNode::callbackCatchTurtle, this, _1, _2));
        this->spawn_turtle_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / this->spawn_frequency_)),
                                                            std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));

        RCLCPP_INFO(this->get_logger(), "Turtle spawner/killer has been started.");
    }

private:
    void callbackCatchTurtle(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                             const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        kill_turtle_threads_.push_back(std::make_shared<std::thread>(
            std::bind(&TurtleSpawnerNode::callKillService, this, request->name)));
        response->success = true;
    }

    void publishAliveTurtles()
    {
        auto newMsg = my_robot_interfaces::msg::TurtleArray();
        newMsg.turtles = this->alive_turtles_;
        this->alive_turtles_publihser_->publish(newMsg);
    }

    // returns random double number in range [0.0, 1.0)
    double randomDouble()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0);
    }

    void spawnNewTurtle()
    {
        this->turtle_counter_ += 1;
        std::string turtle_name = this->turtle_name_prefix_ + std::to_string(this->turtle_counter_);

        double x = randomDouble() * 11.0;
        double y = randomDouble() * 11.0;
        double theta = randomDouble() * 2 * M_PI;

        spawn_turtle_threads_.push_back(std::make_shared<std::thread>(
            std::bind(&TurtleSpawnerNode::callSpawnService, this, turtle_name, x, y, theta)));
    }

    void callSpawnService(std::string turtle_name, double x, double y, double theta)
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        request->name = turtle_name;
        request->x = x;
        request->y = y;
        request->theta = theta;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if (response->name != "")
            {
                auto newTurtle = my_robot_interfaces::msg::Turtle();
                newTurtle.name = response->name;
                newTurtle.x = x;
                newTurtle.y = y;
                newTurtle.theta = theta;
                this->alive_turtles_.push_back(newTurtle);
                this->publishAliveTurtles();
                RCLCPP_INFO(this->get_logger(), "Turtle " + response->name + " is now alive.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void callKillService(std::string turtle_name)
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);

        try
        {
            future.get();
            for (int i = 0; i < (int)(this->alive_turtles_.size()); ++i)
            {
                if (this->alive_turtles_[i].name == turtle_name)
                {
                    this->alive_turtles_.erase(this->alive_turtles_.begin() + i);
                    this->publishAliveTurtles();
                    break;
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    int spawn_frequency_;
    std::string turtle_name_prefix_;

    int turtle_counter_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;

    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publihser_;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_service_;

    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;

    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}