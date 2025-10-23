#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <vector>
//below included for debugging packets
#include <iomanip>
#include <sstream>
//included to check if data is being properly received
#include <cstdint>
#include <boost/system/error_code.hpp>



using boost::asio::ip::udp;

// Define the raw data size and packet structure
constexpr std::size_t DATA_SIZE = 128;         // 8x8 grid of uint16_t = 64 * 2 bytes
constexpr std::size_t PACKET_SIZE = 1 + DATA_SIZE;  // 1 byte sensor ID + grid
constexpr std::size_t NUM_SENSORS = 80;

const uint8_t HEADER[2] = {0xA5, 0x5A};
const int FRAME_SIZE = 128;

class GridReceiver : public rclcpp::Node {
public:
  GridReceiver()
  : Node("grid_receiver"),
    socket_(io_, udp::endpoint(udp::v4(), 5005)),
    rx_buffer_(PACKET_SIZE)
  {
    RCLCPP_INFO(this->get_logger(), "Starting UDP Grid Receiver on port 5005");
    log_connection_info();
    for (size_t i = 0; i < 8; i ++) {
      RCLCPP_INFO(this->get_logger(), "Initializing grid: ");
      pub_.push_back(this->create_publisher<sensor_msgs::msg::Image>("tof_raw_grid_" + std::to_string(i), 10));
    }
    start_receive();
    spin_thread_ = std::thread([this]() { io_.run(); });
  }

  ~GridReceiver() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Grid Receiver");
    io_.stop();
    if (spin_thread_.joinable()) spin_thread_.join();
  }

  void log_connection_info() {
  RCLCPP_INFO(this->get_logger(), "==== UDP Socket Debug Info ====");
  if (socket_.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Socket is open.");
    try {
      auto local_endpoint = socket_.local_endpoint();
      RCLCPP_INFO(this->get_logger(), "Listening on IP: %s, Port: %u",
                  local_endpoint.address().to_string().c_str(), local_endpoint.port());
    } catch (std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Could not get local endpoint: %s", e.what());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Socket is NOT open.");
  }

  RCLCPP_INFO(this->get_logger(), "I/O context running: %s", io_.stopped() ? "false" : "true");
  RCLCPP_INFO(this->get_logger(), "Buffer size: %zu bytes", rx_buffer_.size());
  RCLCPP_INFO(this->get_logger(), "Waiting for packets of size %zu bytes...", PACKET_SIZE);
  RCLCPP_INFO(this->get_logger(), "===============================");
}


private:
  void start_receive() {
    //std::cout << "start receive..." << std::endl;
    
    socket_.async_receive_from(
      boost::asio::buffer(rx_buffer_), sender_,
      [this](boost::system::error_code ec, std::size_t bytes_recvd) {
        if (!ec && bytes_recvd == PACKET_SIZE) {
          RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes from %s:%d",
                       bytes_recvd, sender_.address().to_string().c_str(), sender_.port());
          publish_grid();
        } else if (ec) {
          RCLCPP_WARN(this->get_logger(), "Receive error: %s", ec.message().c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "Unexpected packet size: %zu (expected %zu)",
                      bytes_recvd, PACKET_SIZE);
        }
        start_receive();  // loop
      });
  }

  void publish_grid() {
    if (rx_buffer_.size() < PACKET_SIZE) {
      RCLCPP_WARN(this->get_logger(), "Packet too small to contain expected data");
      return;
    }
  

   
    int sensor_id = rx_buffer_[0];
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = this->now();
    msg.header.frame_id = "tof_sensor_" + std::to_string(sensor_id);
    msg.height = 8;
    msg.width = 8;
    msg.encoding = "mono16";
    msg.step = 16;
    msg.data.resize(DATA_SIZE);

    std::copy(rx_buffer_.begin() + 1, rx_buffer_.begin() + 1 + DATA_SIZE, msg.data.begin());
    pub_[sensor_id]->publish(msg);

     
    //RCLCPP_INFO(this->get_logger(), "Published 8x8 grid from sensor %d", sensor_id);


    //claude debug 
    if (rx_buffer_.size() >= PACKET_SIZE and sensor_id == 0) {

      //RCLCPP_INFO(this->get_logger(), "Published 8x8 grid from sensor %d", sensor_id);



        printf("8x8 ToF Matrix (uint16_t values):\n");
        for (int row = 0; row < 8; ++row) {
            std::string matrix_row = "";
            for (int col = 0; col < 8; ++col) {
                int index = 1 + (row * 8 + col) * 2;  // 1 byte offset + pixel index * 2 bytes
                if (index + 1 < rx_buffer_.size()) {
                    // Combine two bytes into uint16_t (little-endian)
                    uint16_t value = rx_buffer_[index] | (rx_buffer_[index + 1] << 8);
                    char value_str[8];
                    snprintf(value_str, sizeof(value_str), "%5u ", value);
                    matrix_row += value_str;
                } else {
                    matrix_row += "  ??? ";
                }
            }
            printf("Row %d: %s \n", row, matrix_row.c_str());
        }
    }
    //claude debug end 
  }
  

  boost::asio::io_context io_;
  udp::socket socket_;
  udp::endpoint sender_;
  std::vector<uint8_t> rx_buffer_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub_;
  std::thread spin_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridReceiver>());
  rclcpp::shutdown();
  return 0;
}