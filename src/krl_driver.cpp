#include "krl_driver/krl_driver.h"

// Constructor
krlDriver::krlDriver() 
  : Node("krlDriver"), 
    serial_port_("/dev/ttyACM0"), 
    baud_rate_(115200), 
    num_slots_(20), 
    min_publish_time_(0.5)
{
    readConfig();
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&krlDriver::cmd_vel_callback, this, std::placeholders::_1)
    );

    publisher_right_wheel = this->create_publisher<std_msgs::msg::Int64>("right_wheel_rpm", 10);
    publisher_left_wheel = this->create_publisher<std_msgs::msg::Int64>("left_wheel_rpm", 10);
    while(my_serial_ == nullptr){
        try {
        my_serial_ = std::make_unique<serial::Serial>(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
        if (my_serial_ && my_serial_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception opening serial port: %s", e.what());
    }
    }
    
}

void krlDriver::readSerialData() {
    while (true) {
        try {
            while (my_serial_->isOpen()) {
                std::string buffer;
                std::string flag = "A";
                while(flag != "Z"){
                    my_serial_->read(flag,1);
                   // std::cout << "|" << flag[0]<< "|"<<std::endl;
                    if(flag == "Z")
                        break;
                    flag = "";
                }

                //std::cout << "CIKTIK"<<std::endl;
               // std::vector<uint8_t> buffer(num_bytes_to_read);
               // my_serial_->read(buffer,256);
                size_t bytes_read = my_serial_->read(buffer,buffer_size_);
                
                if (bytes_read > 0) {
                    std::cout << "arti eski: " <<buffer[0] <<" ";
                    std::cout << "rpm: " <<buffer[2] << buffer[3] <<" ";
                    std::cout <<"left right: "<< buffer[1] << std::endl;
                    //std::cout << std::dec << std::endl;
                } else {
                    std::cerr << "No data received." << std::endl;
                }
                std::cout << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            if (!my_serial_ || !my_serial_->isOpen()) {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open.");
                return;
            }
        } catch (std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
}


void krlDriver::readConfig(){
    YAML::Node config = YAML::LoadFile("/home/gturover/anadolu_ws/src/krl_driver/params/krl_driver.yaml");
    buffer_size_ = config["buffer_size"].as<int>();

}
void krlDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {


    int32_t num1 = static_cast<int32_t>(msg->linear.x * 10000000);
   int32_t num2 = static_cast<int32_t>(msg->angular.z * 10000000);

    // Packing the data into a byte array
    uint8_t serial_msg[8];
    memcpy(&serial_msg[0], &num1, sizeof(num1));
    memcpy(&serial_msg[4], &num2, sizeof(num2));

    // Unpacking to verify the values
    int32_t unpacked_num1, unpacked_num2;
    memcpy(&unpacked_num1, &serial_msg[0], sizeof(unpacked_num1));
    memcpy(&unpacked_num2, &serial_msg[4], sizeof(unpacked_num2));

     std::cout << "Unpacked values: " << unpacked_num1 << ", " << unpacked_num2 << std::endl;

    // Print the raw bytes
    std::cout << "Serialized bytes: ";
    for (size_t i = 0; i < sizeof(serial_msg); ++i) {
        std::cout << std::hex << static_cast<int>(serial_msg[i]) << " ";
    }
    std::cout << std::endl;
    
size_t bytes_written = my_serial_->write(serial_msg, sizeof(serial_msg));
if (bytes_written == sizeof(serial_msg)) {
    // Success
    std::cout << "All bytes written successfully." << std::endl;
} else {
    // Failure
    std::cerr << "Failed to write all bytes. Bytes written: " << bytes_written << std::endl;
}

}

void krlDriver::startThread() {
    std::thread t(&krlDriver::readSerialData, this);
    t.detach();
}


int krlDriver::calculateRpm(int dcount, double dt) {
    if (dt != 0) {
        return static_cast<int>((dcount / (num_slots_ * (dt / 60))));
    }
    return 0;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto driver = std::make_shared<krlDriver>();

    driver->startThread();
    RCLCPP_INFO(driver->get_logger(), "Start spinning...");

    rclcpp::spin(driver);
    rclcpp::shutdown();
    return 0;
}
