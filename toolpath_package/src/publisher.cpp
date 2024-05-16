#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "processor.h"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        // Define your paths here
       std::vector<std::string> paths = {
        "107 171 108 170 125 170 126 171 128 171 129 172 128 173 107 173 106 174 103 174 102 175 100 175 99 176 96 176 95 177 94 177 91 180 89 180 88 181 87 181 83 185 83 186 81 188 81 189 80 190 80 191 79 192 79 193 78 194 78 196 77 197 77 199 76 200 76 222 77 223 77 226 78 227 78 229 79 230 79 231 80 232 80 235 81 236 81 237 80 238 78 236 78 235 77 234 77 231 76 230 76 229 75 228 75 225 74 224 74 221 73 220 73 209 72 208 72 197 71 196 71 194 69 192 68 192 67 191 66 191 64 189 63 189 62 188 65 185 69 185 70 184 71 185 73 185 71 185 70 184 71 183 73 183 73 182 71 182 70 183 70 184 69 185 66 185 64 183 64 182 66 180 72 180 73 179 76 179 77 178 81 178 82 177 85 177 86 176 88 176 89 175 91 175 92 174 96 174 97 173 100 173 101 172 104 172 105 171",
        "76 138 76 141 75 142 75 147 73 149 72 149 74 151 73 152 73 153 71 155 71 158 72 159 73 158 74 158 76 156 77 157 77 156 79 154 81 154 81 153 83 151 85 153 85 154 85 151 82 151 81 152 81 153 80 154 77 154 74 157 72 157 71 156 73 154 73 152 74 151 73 150 73 149 75 147 75 143 76 142 76 140 77 139 77 138",
        "224 135 223 136 222 136 226 136 227 137 229 137 230 138 234 138 235 139 239 139 240 140 242 140 243 141 244 141 245 142 247 142 248 143 249 143 250 144 250 142 248 142 247 141 243 141 242 140 241 140 240 139 236 139 235 138 233 138 232 137 227 137 225 135",
        "197 132 197 133 196 134 197 134 198 133 204 139 204 140 206 142 207 141 208 142 209 142 210 143 211 143 212 144 212 145 213 145 214 146 218 146 217 146 216 145 214 145 212 143 210 143 208 141 207 141 204 138 204 137 203 137 201 135 201 134 200 133 199 133 198 132",
        "373 121 373 124 372 125 372 148 373 149 373 156 374 157 374 179 373 180 373 181 371 183 371 192 370 193 370 207 369 208 369 213 368 214 368 215 363 220 363 223 360 226 360 239 358 241 358 245 357 246 357 252 356 253 355 252 355 245 355 252 354 253 354 254 353 255 353 256 354 256 355 257 355 259 354 260 354 264 352 266 352 267 351 268 351 270 350 271 350 273 349 274 349 275 348 276 348 278 347 279 347 281 346 282 346 283 345 284 345 285 344 286 344 291 343 292 343 294 342 295 342 297 341 298 341 299 338 302 338 303 336 305 336 306 326 316 325 316 324 317 319 317 317 319 317 321 316 322 316 326 315 327 315 329 314 330 314 331 313 332 313 333 312 334 312 339 311 340 311 343 310 344 310 345 309 346 309 348 308 349 308 351 306 353 306 356 305 357 305 358 304 359 304 360 303 361 303 363 301 365 301 367 299 369 299 370 296 373 296 375 284 387 284 388 283 389 283 427 284 428 284 457 285 458 285 461 286 462 286 469 287 470 287 472 286 473"
    };

        processor = std::make_unique<Processor>(paths);

        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();

        // Process the path and get the result
        int startNode = 2; // Change this to the desired start node
        auto [path, totalDistance] = processor->processPath(startNode);

        // Construct the message string
        std::stringstream ss;
        ss << "TSP Path starting from node " << startNode << ": ";
        for (int node : path) {
            ss << node << " ";
        }
        ss << "\nTotal distance traveled: " << totalDistance;
        message.data = ss.str();

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::unique_ptr<Processor> processor;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
