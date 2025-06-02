using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System;

public class MovementGoal : MonoBehaviour
{
    private ROSConnection ros;
    // private string topicName = "nav2_msgs/action/NavigateToPose";
    [SerializeField] private string topicName = "/goal_pose";
    public GameObject destination;

    [SerializeField] private float messageDelay = 0.5f;
    [SerializeField] private float timeSinceMessage = 0.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    private void Update()
    {
        timeSinceMessage += Time.deltaTime;
        if (timeSinceMessage >= messageDelay)
        {
            // They use a different coordinate system
            PointMsg position = new PointMsg(
                destination.transform.position.z,
                -destination.transform.position.x,
                destination.transform.position.y
            );
            QuaternionMsg orientation = new QuaternionMsg(
                destination.transform.rotation.x,
                destination.transform.rotation.y,
                destination.transform.rotation.z,
                destination.transform.rotation.w
            );
            PoseMsg pose = new PoseMsg(position, orientation);

            HeaderMsg header = new HeaderMsg();
            PoseStampedMsg msg = new PoseStampedMsg(header, pose);
            ros.Publish(topicName, msg);
            // byte[] bytes = msg.Serialize();
            // string str = System.Text.Encoding.ASCII.GetString(bytes);
            // print($"Sending message to topic {topicName} with payload {str}.");
            timeSinceMessage = 0.0f;
        }

    }
}
