using System.Collections;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class Rob3Agent : Agent
{
    public Rigidbody mobileBase;
    public HingeJoint arm1Joint;
    public HingeJoint arm2Joint;
    public Transform targetPosition;

    public Transform wall;
    public Transform wall2;
    public Transform wall3;
    public Transform wall4;

    public Transform obs1;
    public Transform obs2;


    private float targetArm1Angle = 0f;
    private float targetArm2Angle = 0f;

    public float baseSpeed = 5f;
    public float forceMultiplier = 10;

    public override void OnEpisodeBegin()
    {
        // Reset Rigidbody velocities
        mobileBase.linearVelocity = Vector3.zero;
        mobileBase.angularVelocity = Vector3.zero;

        // Reset the base position
        mobileBase.transform.position = new Vector3(3.5f, 0.05f, 0);  // Ensure it's above the floor
        //mobileBase.transform.rotation = Quaternion.Euler(0, 0, 0); // Ensure it's oientation is right

        // Disable joint motors temporarily
        ResetJointMotor(arm1Joint);
        ResetJointMotor(arm2Joint);

        // Randomize target position within a 5x5 area
        targetPosition.localPosition = new Vector3(Random.Range(0, -2.5f), 0.05f, Random.Range(-2.5f, 2.5f));

        // Reset arm angles
        targetArm1Angle = 0f;
        targetArm2Angle = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Collect observations for agent's position, velocity, joint angles, and target position
        sensor.AddObservation(mobileBase.transform.position.x);  // Base x position
        sensor.AddObservation(mobileBase.transform.position.y);  // Base y position
        sensor.AddObservation(mobileBase.transform.position.z);  // Base z position
        sensor.AddObservation(targetPosition.localPosition.x);   // Target x position
        sensor.AddObservation(targetPosition.localPosition.y);   // Target y position
        sensor.AddObservation(targetPosition.localPosition.z);   // Target z position
        sensor.AddObservation(mobileBase.linearVelocity.x);            // Base x velocity
        sensor.AddObservation(mobileBase.linearVelocity.z);            // Base z velocity
        sensor.AddObservation(arm1Joint.angle);                  // Arm1 joint angle
        sensor.AddObservation(arm2Joint.angle);                  // Arm2 joint angle
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");  // Control base movement (x)
        continuousActionsOut[1] = Input.GetAxis("Vertical");    // Control base movement (z)
        continuousActionsOut[2] = Input.GetAxis("Arm1");        // Control arm1 joint angle
        continuousActionsOut[3] = Input.GetAxis("Arm2");        // Control arm2 joint angle
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions: [base movement (x), base movement (z), arm1 joint control, arm2 joint control]

        // Control the mobile base
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.z = actionBuffers.ContinuousActions[1];
        mobileBase.AddForce(controlSignal * forceMultiplier);

        // Update joint target angles based on action values
        targetArm1Angle += actionBuffers.ContinuousActions[2];
        targetArm2Angle += actionBuffers.ContinuousActions[3];

        // Clamp joint angles within safe limits
        targetArm1Angle = Mathf.Clamp(targetArm1Angle, -90f, 90f);
        targetArm2Angle = Mathf.Clamp(targetArm2Angle, -90f, 90f);

        // Control joints with motors
        SetJointMotor(arm1Joint, targetArm1Angle);
        SetJointMotor(arm2Joint, targetArm2Angle);

        // Calculate rewards
        float distanceToTarget = Vector3.Distance(mobileBase.transform.position, targetPosition.position);

        // Reward for reaching and maintaining the target
        if (distanceToTarget < 1.4f)
        {
            AddReward(1.0f);  // Significant reward for reaching the target
            EndEpisode();
        }
        else
        {
            // Penalize for distance to target (negative reward proportional to distance)
            AddReward(-distanceToTarget * 0.01f);
        }

        // Reward for joint stability
        float arm1Error = Mathf.Abs(targetArm1Angle - 0f);  // Deviation from desired angle
        float arm2Error = Mathf.Abs(targetArm2Angle - 0f);
        AddReward(-arm1Error * 0.01f);  // Penalize deviations
        AddReward(-arm2Error * 0.01f);

        // Penalize excessive joint velocities
        AddReward(-Mathf.Abs(arm1Joint.velocity) * 0.001f);
        AddReward(-Mathf.Abs(arm2Joint.velocity) * 0.001f);

        // Check if the agent touches any wall
        //if (TouchesWall())
        //{
        //    SetReward(-1.0f);  // Large penalty for hitting a wall
        //    EndEpisode();
        //}

        //// Check if the agent touches any wall
        //if (TouchesObs())
        //{
        //    SetReward(-2.0f);  // Large penalty for hitting obs
        //    EndEpisode();
        //}
    }

    private bool TouchesWall()
{
    // Calculate distances to walls and check for collisions
    float distanceToWall1 = Mathf.Abs(mobileBase.transform.position.x - wall.position.x);
    float distanceToWall2 = Mathf.Abs(mobileBase.transform.position.x - wall2.position.x);
    float distanceToWall3 = Mathf.Abs(mobileBase.transform.position.z - wall3.position.z);
    float distanceToWall4 = Mathf.Abs(mobileBase.transform.position.z - wall4.position.z);

    // Check if the agent is very close to any wall
    float threshold = 0.1f; // Define a small threshold for wall proximity
    if (distanceToWall1 < threshold || distanceToWall2 < threshold || distanceToWall3 < threshold || distanceToWall4 < threshold)
    {
        return true;
    }

    return false;
}

private bool TouchesObs()
{
    // Calculate distances to walls and check for collisions
    float distanceToobs1 = Mathf.Abs(mobileBase.transform.position.x - obs1.position.x);
    float distanceToobs2 = Mathf.Abs(mobileBase.transform.position.x - obs2.position.x);

    // Check if the agent is very close to any wall
    float threshold = 0.01f; // Define a small threshold for wall proximity
    if (distanceToobs1 < threshold || distanceToobs2 < threshold)
    {
        return true;
    }

    return false;
}

private void SetJointMotor(HingeJoint joint, float targetAngle)
    {
        // Configure motor settings
        var motor = joint.motor;
        motor.targetVelocity = (targetAngle - joint.angle) * 10f;  // Smooth control
        motor.force = 10f;  // Motor force
        motor.freeSpin = false;
        joint.motor = motor;
        joint.useMotor = true;
    }

    private void ResetJointMotor(HingeJoint joint)
    {
        joint.useMotor = false;  // Disable motor
    }
}

