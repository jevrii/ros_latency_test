#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"

#include <sstream>
#include <vector>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

std::vector<double> v;
double latency;

void callback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
  latency = ros::Time::now().toSec() - v[-(msg->data)];
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher ping_pub = n.advertise<std_msgs::Int32>("ping1", 1);
  ros::Publisher latency_pub = n.advertise<std_msgs::Float64>("latency", 1);
  ros::CallbackQueue queue_1;
  n.setCallbackQueue(&queue_1);
  ros::Subscriber sub = n.subscribe("ping4", 10, callback);
  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("test", false);
  std_srvs::Trigger srv;

  int r = 100;

  ros::Rate loop_rate(r);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Int32 msg;

    msg.data = count;

    ROS_INFO("%d", count);

    std_msgs::Float64 msg2;
    
    if (true) {
      v.push_back(ros::Time::now().toSec());

      //   /**
      //    * The publish() function is how you send messages. The parameter
      //    * is the message object. The type of this object must agree with the type
      //    * given as a template parameter to the advertise<>() call, as was done
      //    * in the constructor above.
      //    */
        ping_pub.publish(msg);

      //   // ros::spinOnce();

        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.0025));
        queue_1.callAvailable(ros::WallDuration(0.005));
        msg2.data = latency;

        latency_pub.publish(msg2);
    }
    else {
      ros::Time start = ros::Time::now();
      client.call(srv);
      msg2.data = (ros::Time::now() - start).toSec();

      latency_pub.publish(msg2);
    }

    loop_rate.sleep();
    ++count;
  }
// spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}