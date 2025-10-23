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
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/point_field.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>



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
  

   
    /*
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

    RCLCPP_INFO(this->get_logger(), "Published 8x8 grid from sensor %d", sensor_id); 
    */

    std::vector<uint16_t> received_data;

    //std::copy(rx_buffer_.begin() + 1, rx_buffer_.begin() + 1 + DATA_SIZE, recevied_data.begin());

    int sensor_id = rx_buffer_[0];

    auto msg = sensor_msgs::msg::PointCloud2(); 

    /*sensor_msgs::PointCloud2Modifier modifier(msg);

    modifier.[PointField(name='x',1, datatype=PointField.FLOAT32, count=1),
              PointField(name='y',1, datatype=PointField.FLOAT32, count=1),
              PointField(name='z',1, datatype=PointField.FLOAT32, count=1),
              PointField(name='depth', 1, datatype=PointField.FLOAT32, count=1)];
    
    msg.is_dense = true;
    msg.header.stamp = this->now();
    msg.header.frame_id = "fr3_link5/sensor_" + std::to_string(sensor_id);
    msg.height = 8;
    msg.width = 8;

    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width * msg.height;
    msg.data.resize(msg.row_step);

    sensor_msgs::PointCloud2Iterator<float> iterX(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterDepth(msg, "depth");



    {
      *iterX = //Your x data
      *iterY = //Your y data
      *iterZ = //Your z data
      *iterDepth = //Your intensity data

      // Increment the iterators
      ++iterX;
      ++iterY;
      ++iterZ;
      ++iterDepth;
    }*/
   
    
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