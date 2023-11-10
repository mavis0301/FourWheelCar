using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using System.Linq;

public class Robot : MonoBehaviour
{   


    public struct Action {
        public List<float> voltage;
    }

    public MotionSensor baseLinkM;
    public List<MotionSensor>  wheelM;

    // List<Motor> motorList;
    List<MotorMoveForward> motorListMF;

    public TrailRenderer trailRenderer;


    //test
    public Vector3 rot;

    public State ROS2State;


    Vector3 targetPosition;


    const float DEG2RAD = 0.01745329251f;


    public LidarSensor lidar;

    void Awake()
    {   
        baseLinkM = Util.GetOrAddComponent<MotionSensor>(transform, "base_link");
        
        
        wheelM = new List<MotionSensor>() {
            Util.GetOrAddComponent<MotionSensor>(transform, "front_right"),
            Util.GetOrAddComponent<MotionSensor>(transform, "front_left"),
            Util.GetOrAddComponent<MotionSensor>(transform, "back_right"),
            Util.GetOrAddComponent<MotionSensor>(transform, "back_left"), 
        };

        motorListMF = new List<MotorMoveForward>() {
            Util.GetOrAddComponent<MotorMoveForward>(transform, "front_right"),
            Util.GetOrAddComponent<MotorMoveForward>(transform, "front_left"),
            Util.GetOrAddComponent<MotorMoveForward>(transform, "back_right"),
            Util.GetOrAddComponent<MotorMoveForward>(transform, "back_left"),
        };
   

    
        trailRenderer = GetComponentInChildren<TrailRenderer>();
    }

    public State GetState(Vector3 newTarget)
    {   
        Vector3 carPos = baseLinkM.x;
        Vector3 carVel = baseLinkM.v;
        Vector3 carAngV = baseLinkM.AngularV;
        Quaternion carQ = baseLinkM.q;

        Vector3 angVFR = wheelM[0].AngularV;
        Vector3 angVFL = wheelM[1].AngularV;
        Vector3 angVBR = wheelM[2].AngularV;
        Vector3 angVBL = wheelM[3].AngularV;

        List<float> range = lidar.GetRange();
        // Debug.Log("min range: " + range);

        var rangeDirection = lidar.GetRangeDirection(); //list
        for(int i = 0; i < rangeDirection.Count; i++){
            rangeDirection[i] = ToRosVec(rangeDirection[i]);
        }

        State ROS2State = new State(){
            // targetPosition = newTarget,
            // pathPositionClosest = new Vector3(0,0,0),
            // pathPositionSecondClosest = new Vector3(0,0,0),
            // pathPositionFarthest = new Vector3(0,0,0),
            carPosition = carPos,
            // carVelocity = carVel,
            // carAugularVelocity = carAngV,
            // carQuaternion = carQ,
            // wheelAngularVelocityLeftBack = angVLB,
            // wheelAngularVelocityLeftFront = angVLF,
            // wheelAngularVelocityRightBack = angVRB,
            // wheelAngularVelocityRightFront = angVRF,
            // wheelQuaternionLeftFront = qLF,
            // wheelQuaternionRightFront = qRF,
            // minRange = range,
            // minRangeDirection = rangeDirection,

            ROS2TargetPosition = ToRosVec(newTarget),
            ROS2PathPositionClosest = new Vector3(0,0,0),
            ROS2PathPositionSecondClosest = new Vector3(0,0,0),
            ROS2PathPositionFarthest = new Vector3(0,0,0),
            ROS2CarPosition = ToRosVec(carPos),
            ROS2CarVelocity = ToRosVec(carVel),
            ROS2CarAugularVelocity = ToRosVec(carAngV),
            ROS2CarQuaternion = ToRosQuaternion(carQ),

            ROS2WheelAngularVelocityRightFront = ToRosVec(angVFR),
            ROS2WheelAngularVelocityLeftFront = ToRosVec(angVFL),
            ROS2WheelAngularVelocityRightBack = ToRosVec(angVBR),
            ROS2WheelAngularVelocityLeftBack = ToRosVec(angVBL),

            ROS2Range = range.ToArray(),
            // ROS2MinRangeDirection = ToRosVec(rangeDirection),
            ROS2RangePosition = rangeDirection.ToArray(),
            

        };
      

        // (double, double, double) tmp = EulerFromQuaternion(baseLinkM.q);
        // Vector3 tmp2 = new Vector3((float)tmp.Item1/DEG2RAD, (float)tmp.Item2/DEG2RAD, (float)tmp.Item3/DEG2RAD);
        // Debug.Log("car before q " + tmp2);
        // Vector3 tmp5 = baseLinkM.q.eulerAngles;
        // Debug.Log("car before eu " + tmp5);
        // (double, double, double) tmp3 = EulerFromQuaternion(baseLinkM.ROS2Quaternion);
        // Vector3 tmp4 = new Vector3((float)tmp3.Item1/DEG2RAD, (float)tmp3.Item2/DEG2RAD, (float)tmp3.Item3/DEG2RAD);
        // Debug.Log("car after " + tmp4);

        // var tmp = wheelBaseM[0].theta.y / DEG2RAD;
        // print("wheel angular v " + angVLB.x.ToString("F5"));
  
        return ROS2State;
    
    }

    public State UpdatePath(State state, Vector3 p0, Vector3 p1, Vector3 p2) 
    {
        state.ROS2PathPositionClosest = ToRosVec(p0);
        state.ROS2PathPositionSecondClosest = ToRosVec(p1);
        state.ROS2PathPositionFarthest = ToRosVec(p2);

        return state;
    }

    

    public void DoAction(Action action)
    {   
        ////rear engine
        // motorListMF[0].SetVoltage(action[0]);
        // motorListMF[1].SetVoltage(action[1]);
        
        motorListMF[0].SetVoltage((float)action.voltage[0]);
        motorListMF[1].SetVoltage((float)action.voltage[1]);

        ////rear engine
        motorListMF[2].SetVoltage((float)action.voltage[2]);
        motorListMF[3].SetVoltage((float)action.voltage[3]);
    }

    public float getTargetAngle(Vector2 pos, Vector2 targetPos)
    {
        Vector2 v = targetPos - pos;
        Vector2 up = new Vector2(0, 1);
        float rad = getAngle(v, up);
        if(v[0] < 0)
            rad = Mathf.PI*2 - rad;
        return rad;
    }

    public float getAngle(Vector2 v1, Vector2 v2)
    {
        v1.Normalize();
        v2.Normalize();
        return Mathf.Acos(Vector2.Dot(v1, v2));
    }

    static (double, double, double) EulerFromQuaternion(Quaternion orientation)
    {
        /*
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        */

        double x = orientation.x;
        double y = orientation.y;
        double z = orientation.z;
        double w = orientation.w;

        double t0 = +2.0 * (w * x + y * z);
        double t1 = +1.0 - 2.0 * (x * x + y * y);
        double roll_x = Math.Atan2(t0, t1);

        double t2 = +2.0 * (w * y - z * x);
        t2 = +1.0 > t2 ? +1.0 : t2;
        t2 = -1.0 < t2 ? -1.0 : t2;
        double pitch_y = Math.Asin(t2);

        double t3 = +2.0 * (w * z + x * y);
        double t4 = +1.0 - 2.0 * (y * y + z * z);
        double yaw_z = Math.Atan2(t3, t4);

        return (roll_x, pitch_y, yaw_z);
    }

    Vector3 ToRosVec(Vector3 position)
    {
        // Debug.Log("before " + position.ToString("F5"));
        PointMsg ROS2Position = position.To<FLU>();
        position = new Vector3((float)ROS2Position.x, (float)ROS2Position.y, (float)ROS2Position.z);
        // Debug.Log("after " + position.ToString("F5"));

        return position;
    }

    Quaternion ToRosQuaternion(Quaternion quaternion) 
    {   
        // Debug.Log("before " + quaternion.ToString("F5"));

        QuaternionMsg ROS2Quaternion = quaternion.To<FLU>();
        quaternion = new Quaternion((float)ROS2Quaternion.x, (float)ROS2Quaternion.y, (float)ROS2Quaternion.z, (float)ROS2Quaternion.w);
        // Debug.Log("after " + quaternion.ToString("F5"));

        quaternion.To<FLU>();   
        return quaternion;
    }





}

