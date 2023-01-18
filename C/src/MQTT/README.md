# Interfacing Lingua Franca with MQTT

MQTT is a popular publish-and-subscribe framework using in IoT and other applications. Using MQTT, program publish data on "topics" and subscribe to "topics". A broker, pointed to by a URL, disseminates published topics to all subscribers. The main challenge with using this with LF is understanding at what tags subscription data will appear. It is possible, albeit challenging, to get deterministic behavior, but only under some constraints.

This directory contains two library reactors, [`MQTTPublisher`](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/MQTT/lib/MQTTPublisher.lf) and [`MQTTSubscriber`](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/MQTT/lib/MQTTSubscriber.lf), that publish and subscribe to string data and handle assignment of tags at the subscriber based on the tag at the publisher. These are designed to be used with the C target.

A few examples illustrate the subtleties and provide guidance on how to get repeatable behavior. To compile and run these, you need to install some software (see [Prerequisites](#prerequisites)). 

## Summary of the Examples

* **[MQQTLogical](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/MQTT/MQTTLogical.lf)**: This LF program periodically publishes on a topic and, in a different part of the program, subscribes to the same topic. The program has no other activity, so as long as the time between publishing of events is larger than the total latency through the MQTT broker, the subscriber will see messages one microstep later than the publisher.  By setting a positive `offset` at the subscriber, you can increase the logical time of the received message and get a zero microstep. This gives behavior similar to an LF connection with an `after` delay.

* **[MQQTPhysical](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/MQTT/MQTTPhysical.lf)**: This LF program also periodically publishes on a topic and, in a different part of the program, subscribes to the same topic. The `use_physical_time` parameter of `MQTTSubscriber` is set to true, so unlike `MQQTLogical`, the timestamp at the receiving end will be nondeterministically determined from the local physical clock. The difference between the publisher's logical time and the subscriber's logical time is a reasonable measure of the latency through the MQTT broker. This gives behavior similar to an LF [physical connection](https://www.lf-lang.org/docs/handbook/composing-reactors?target=c#physical-connections).

* **[MQQTDistributed](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/MQTT/MQTTDistributed.lf)**: This is a federated LF program consisting of two unconnected federates that communicate via MQTT. Like `MQTTLogical`, there is no other activity in this program, so the subscriber's timestamps will deterministically match those of the publisher. Unlike `MQTTLogical`, however, the microstep will be zero at the subscriber end. Also, the tags will be deterministic at the receiving end regardless of the communication latency because the receiving federate has no reason to advance its logical time unless it receives an MQTT subscription message. You can change the `use_physical_time` parameter of the `MQTTSubscriber` to `true` to get a (nondeterministic) physical connection, similar to `MQTTPhysical`. 

* **[MQQTLegacy](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/MQTT/MQTTLegacy.lf)**: This program illustrates how to interface to a legacy MQTT service that has no connection with Lingua Franca. The `LegacyPublisher` reactor publishes messages every 5s that can be listened to by any legacy service, such as the command-line `mosquito_sub` utility. In addition, the `Listener` reactor can received messages published by any legacy MQTT publisher, such as the command-line `moquito_pub` utility.

## Prerequisites:

To get this example to compile, you will need to install the [Eclipse Paho MQTT C client library,](https://github.com/eclipse/paho.mqtt.c), which requires that you first install
[openSSL](https://github.com/openssl/openssl.git) (see [https://www.openssl.org](https://www.openssl.org). To run the compiled code, you need an MQTT broker to be running. For example, the [Mosquitto Eclipse project](https://mosquitto.org/download/) provides one. On a Mac, you can use homebrew to install the Mosquitto broker:

    brew install mosquitto

To start the broker and test it, do this:

1. Start the broker in the background:

    > mosquitto &
    
2. Start a command-line subscriber:

    > mosquitto_sub -v -t 'test/topic'
    
3. In another terminal window, publish a message:

    > mosquitto_pub -t 'test/topic' -m 'Hello World'

## Implementation

The [`MQTTPublisher`](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/MQTT/lib/MQTTPublisher.lf) and [`MQTTSubscriber`](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/MQTT/lib/MQTTSubscriber.lf) reactor use the [Paho MQTT Client Library](https://github.com/eclipse/paho.mqtt.c) (see the [documentation](https://www.eclipse.org/paho/files/mqttdoc/MQTTClient/html/_m_q_t_t_client_8h.html)).
