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
    public Transform wall, wall2, wall3, wall4;
    public Transform obs1, obs2;

    private float targetArm1Angle = 0f;
    private float targetArm2Angle = 0f;
    public float forceMultiplier = 10;

    public override void OnEpisodeBegin()
    {
        // Reset Rigidbody velocities
        mobileBase.linearVelocity = Vector3.zero;
        mobileBase.angularVelocity = Vector3.zero;

        // Find a safe spawn position
        Vector3 spawnPosition;
        spawnPosition = new Vector3(Random.Range(-2.5f, 2.5f), 0.5f, Random.Range(-2.5f, 2.5f));
           
        // Set position and reset orientation
        mobileBase.transform.position = spawnPosition;
        mobileBase.transform.rotation = Quaternion.Euler(0,0,0);

        // Reset arm angles and disable joint motors temporarily
        ResetJointMotor(arm1Joint);
        ResetJointMotor(arm2Joint);
        targetArm1Angle = 0f;
        targetArm2Angle = 0f;

        // Disable collisions temporarily
        //StartCoroutine(DisableCollisionsTemporarily());

        // Randomize target position within a 5x5 area
        targetPosition.localPosition = new Vector3(Random.Range(-2.5f, 2.5f), 0.05f, Random.Range(-2.5f, 2.5f));
    }

    private IEnumerator DisableCollisionsTemporarily()
    {
        // Disable collisions
        DisableCollisions();

        // Wait for 0.5 seconds (adjust as needed for stabilization)
        yield return new WaitForSeconds(0.1f);

        // Re-enable collisions
        EnableCollisions();
    }

    private void DisableCollisions()
    {
        // Example: Disable collision detection for the mobile base and other components
        mobileBase.detectCollisions = false;
        arm1Joint.connectedBody.detectCollisions = false;
        arm2Joint.connectedBody.detectCollisions = false;
    }

    private void EnableCollisions()
    {
        // Re-enable collision detection
        mobileBase.detectCollisions = true;
        arm1Joint.connectedBody.detectCollisions = true;
        arm2Joint.connectedBody.detectCollisions = true;
    }

    private void ResetJointMotor(HingeJoint joint)
    {
        joint.useMotor = false; // Disable motor
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float baseX = mobileBase.transform.position.x;
        float baseY = mobileBase.transform.position.y;
        float baseZ = mobileBase.transform.position.z;
        float targetX = targetPosition.localPosition.x;
        float targetY = targetPosition.localPosition.y;
        float targetZ = targetPosition.localPosition.z;
        float baseVelocityX = mobileBase.linearVelocity.x;
        float baseVelocityZ = mobileBase.linearVelocity.z;
        float arm1Angle = arm1Joint.angle;
        float arm2Angle = arm2Joint.angle;

        // Debug each observation
        //Debug.Log($"Base Pos: {baseX}, {baseY}, {baseZ}, Target Pos: {targetX}, {targetY}, {targetZ}, Vel: {baseVelocityX}, {baseVelocityZ}, Angles: {arm1Angle}, {arm2Angle}");

        // Add observations
        sensor.AddObservation(baseX);
        sensor.AddObservation(baseY);
        sensor.AddObservation(baseZ);
        sensor.AddObservation(targetX);
        sensor.AddObservation(targetY);
        sensor.AddObservation(targetZ);
        sensor.AddObservation(baseVelocityX);
        sensor.AddObservation(baseVelocityZ);
        sensor.AddObservation(arm1Angle);
        sensor.AddObservation(arm2Angle);
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
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.z = actionBuffers.ContinuousActions[1];
        mobileBase.AddForce(controlSignal * forceMultiplier);

        // Update joint target angles based on action values
        targetArm1Angle += actionBuffers.ContinuousActions[2];
        targetArm2Angle += actionBuffers.ContinuousActions[3];
        targetArm1Angle = Mathf.Clamp(targetArm1Angle, -90f, 90f);
        targetArm2Angle = Mathf.Clamp(targetArm2Angle, -90f, 90f);

        SetJointMotor(arm1Joint, targetArm1Angle);
        SetJointMotor(arm2Joint, targetArm2Angle);

        // Reward calculations
        float distanceToTarget = Vector3.Distance(mobileBase.transform.position, targetPosition.position);
        if (distanceToTarget < 1.4f)
        {
            AddReward(1.0f);
            EndEpisode();
        }

        // Fell off platform
        else if (mobileBase.transform.localPosition.y < 0)
        {
            EndEpisode();
        }

        else
        {
            AddReward(-distanceToTarget * 0.01f);
        }
    }

    private void SetJointMotor(HingeJoint joint, float targetAngle)
    {
        var motor = joint.motor;
        motor.targetVelocity = (targetAngle - joint.angle) * 10f;
        motor.force = 10f;
        motor.freeSpin = false;
        joint.motor = motor;
        joint.useMotor = true;
    }
}
