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

    [SerializeField] private GameObject baseFootprint;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    public void NavigateToObject(GameObject destination)
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

    }

    public float DistanceToObject(GameObject destination) =>
        Vector3.Distance(destination.transform.position, baseFootprint.transform.position);
}
