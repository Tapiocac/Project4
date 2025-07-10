# Fast DDS HelloWorld Demo

本示例演示如何基于 eProsima Fast DDS 实现一个最简单的发布-订阅（Pub/Sub）Demo。

1. 运行示例：
   - 启动订阅者：
     ```bash
     ./hello_subscriber
     ```
   - 启动发布者：
     ```bash
     ./hello_publisher
     ```

你将看到发布者每秒发送一条消息，订阅者打印接收到的序号和内容。
