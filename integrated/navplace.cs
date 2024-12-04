using System.Collections;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using static UnityEngine.GraphicsBuffer;

public class navplace : Agent
{
    public Rigidbody mobileBase;
    //public HingeJoint arm1Joint;
    //public HingeJoint arm2Joint;
    public Transform targetPosition;
    //public Transform wall, wall2, wall3, wall4;
    //public Transform obs1, obs2, obs3, obs4;

    //private float targetArm1Angle = 0f;
    //private float targetArm2Angle = 0f;
    public float forceMultiplier = 75;
    private float previousDistanceToTarget;

    public override void OnEpisodeBegin()
    {
        // Reset Rigidbody velocities
        mobileBase.linearVelocity = Vector3.zero;
        mobileBase.angularVelocity = Vector3.zero;

        // Find a safe spawn position
        Vector3 spawnPosition;
        spawnPosition = new Vector3(10f, 0f, 0f);

        // Set position and reset orientation
        mobileBase.transform.position = spawnPosition;
        mobileBase.transform.rotation = Quaternion.Euler(0, 0, 0);

        //Previous distance
        previousDistanceToTarget = Vector3.Distance(mobileBase.transform.position, targetPosition.position);

        // Reset arm angles and disable joint motors temporarily
        //ResetJointMotor(arm1Joint);
        //ResetJointMotor(arm2Joint);
        //targetArm1Angle = 0f;
        //targetArm2Angle = 0f;

        // Disable collisions temporarily
        //StartCoroutine(DisableCollisionsTemporarily());

        // Randomize target position within a 5x5 area
        //targetPosition.localPosition = new Vector3(Random.Range(-12f, -5f), 0.05f, Random.Range(-12f, 12f));
        //targetPosition.localPosition = new Vector3(Random.Range(-5.5f, 5.5f), 0.05f, Random.Range(-5.5f, 5.5f)); 
        targetPosition.localPosition = new Vector3(-5f, 0.5f, 0f);

    }

    //private IEnumerator DisableCollisionsTemporarily()
    //{
    //    // Disable collisions
    //    DisableCollisions();

    //    // Wait for 0.5 seconds (adjust as needed for stabilization)
    //    yield return new WaitForSeconds(0.1f);

    //    // Re-enable collisions
    //    EnableCollisions();
    //}

    //private void DisableCollisions()
    //{
    //    // Example: Disable collision detection for the mobile base and other components
    //    mobileBase.detectCollisions = false;
    //    arm1Joint.connectedBody.detectCollisions = false;
    //    arm2Joint.connectedBody.detectCollisions = false;
    //}

    //private void EnableCollisions()
    //{
    //    // Re-enable collision detection
    //    mobileBase.detectCollisions = true;
    //    arm1Joint.connectedBody.detectCollisions = true;
    //    arm2Joint.connectedBody.detectCollisions = true;
    //}

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
        //float arm1Angle = arm1Joint.angle;
        //float arm2Angle = arm2Joint.angle;

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
        //sensor.AddObservation(arm1Angle);
        //sensor.AddObservation(arm2Angle);
    }

    //public override void OnEpisodeBegin()
    //{
    //    if (this.transform.localPosition.y < 0)
    //    {
    //        // If the Agent fell, zero its momentum
    //        this.rBody.angularVelocity = Vector3.zero;
    //        this.rBody.velocity = Vector3.zero;
    //        this.transform.localPosition = new Vector3(0, 0.5f, 0);
    //    }

    //    // Move the target to a new spot
    //    Target.localPosition = new Vector3(Random.value * 8 - 4,
    //                                       0.5f,
    //                                       Random.value * 8 - 4);
    //}

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Arm1");  // Control base movement (x)
        continuousActionsOut[1] = Input.GetAxis("Arm2");    // Control base movement (z)
        //continuousActionsOut[2] = Input.GetAxis("Arm1");        // Control arm1 joint angle
        //continuousActionsOut[3] = Input.GetAxis("Arm2");        // Control arm2 joint angle
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions: [base movement (x), base movement (z), arm1 joint control, arm2 joint control]
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.z = actionBuffers.ContinuousActions[1];
        mobileBase.AddForce(controlSignal * forceMultiplier);

        // Update joint target angles based on action values
        //targetArm1Angle += actionBuffers.ContinuousActions[2];
        //targetArm2Angle += actionBuffers.ContinuousActions[3];
        //targetArm1Angle = Mathf.Clamp(targetArm1Angle, -90f, 90f);
        //targetArm2Angle = Mathf.Clamp(targetArm2Angle, -90f, 90f);

        //SetJointMotor(arm1Joint, targetArm1Angle);
        //SetJointMotor(arm2Joint, targetArm2Angle);

        // Reward calculations
        float distanceToTarget = Vector3.Distance(mobileBase.transform.position, targetPosition.localPosition);
        float currentDistanceToTarget = Vector3.Distance(mobileBase.transform.position, targetPosition.position);
        Debug.Log($"Dist to target: {distanceToTarget}");

        if (distanceToTarget < 2.5f)
        {
            AddReward(1.0f);
            EndEpisode();
        }

        //AddReward(- distanceToTarget * 0.01f);
        //Reward based on movement towards the target
        if (currentDistanceToTarget < previousDistanceToTarget)
        {
            SetReward(1f / distanceToTarget); // Positive reward for moving closer
        }
        else
        {
            SetReward(-0.0001f); // Penalty for moving farther away
        }

        // Update the previous distance for the next step
        //previousDistanceToTarget = currentDistanceToTarget;



        // Reward for joint stability
        //float arm1Error = Mathf.Abs(targetArm1Angle - 0f);  // Deviation from desired angle
        //float arm2Error = Mathf.Abs(targetArm2Angle - 0f);
        //AddReward(-arm1Error * 0.0001f);  // Penalize deviations
        //AddReward(-arm2Error * 0.0001f);

        // Penalize excessive joint velocities
        //AddReward(-Mathf.Abs(arm1Joint.velocity) * 0.001f);
        //AddReward(-Mathf.Abs(arm2Joint.velocity) * 0.001f);

        //if (TouchesWall())
        //{
        //    AddReward(-1.0f);  // Large penalty for hitting a wall
        //    EndEpisode();
        //}

        ////Check if the agent touches any wall
        //if (TouchesObs())
        //{
        //    AddReward(-1.0f);  // Large penalty for hitting obs
        //    EndEpisode();
        //}

        if (mobileBase.transform.position.y < 0)
        {
            EndEpisode();
        }

        //void OnTriggerEnter(Collider other)
        //{
        //    // Check if the agent touches any of the obstacles
        //    if (other.CompareTag("Obstacle"))
        //    {
        //        SetReward(-2.0f);  // Large penalty for hitting an obstacle
        //        EndEpisode();      // End the episode after collision
        //    }
        //}


    }

    private void SetJointMotor(HingeJoint joint, float targetAngle)
    {
        var motor = joint.motor;
        motor.targetVelocity = (targetAngle - joint.angle) * 50f;
        motor.force = 50f;
        motor.freeSpin = false;
        joint.motor = motor;
        joint.useMotor = true;
    }

    //private bool TouchesWall()
    //{
    //    // Calculate distances to walls and check for collisions
    //    float distanceToWall1 = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.x) - Mathf.Abs(wall.position.x));
    //    float distanceToWall2 = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.x) - Mathf.Abs(wall2.position.x));
    //    float distanceToWall3 = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.z) - Mathf.Abs(wall3.position.z));
    //    float distanceToWall4 = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.z) - Mathf.Abs(wall4.position.z));

    //    // Check if the agent is very close to any wall
    //    float threshold = 0.5f; // Define a small threshold for wall proximity
    //    if (distanceToWall1 < threshold || distanceToWall2 < threshold || distanceToWall3 < threshold || distanceToWall4 < threshold)
    //    {
    //        return true;
    //    }

    //    return false;
    //}

    //private bool TouchesObs()
    //{
    //    // Calculate distances to walls and check for collisions
    //    float distanceToobs1x = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.x) - Mathf.Abs(obs1.position.x));
    //    float distanceToobs2x = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.x) - Mathf.Abs(obs2.position.x));
    //    float distanceToobs3x = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.x) - Mathf.Abs(obs3.position.x));
    //    float distanceToobs4x = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.x) - Mathf.Abs(obs4.position.x));

    //    float distanceToobs1z = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.z) - Mathf.Abs(obs1.position.z));
    //    float distanceToobs2z = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.z) - Mathf.Abs(obs2.position.z));
    //    float distanceToobs3z = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.z) - Mathf.Abs(obs3.position.z));
    //    float distanceToobs4z = Mathf.Abs(Mathf.Abs(mobileBase.transform.position.z) - Mathf.Abs(obs4.position.z));

    //    // Check if the agent is very close to any wall
    //    float threshold1 = 2.75f;
    //    float threshold2 = 0.75f; // Define a small threshold for wall proximity
    //    if (distanceToobs1x < threshold1 && distanceToobs1z < threshold2 || distanceToobs2x < threshold1 && distanceToobs2z < threshold2 || distanceToobs3z < threshold1 && distanceToobs3x < threshold2 || distanceToobs4z < threshold1 && distanceToobs4x < threshold2)
    //    {
    //        return true;
    //    }
    //    //else if (distanceToobs1z < threshold2 || distanceToobs2z < threshold2 || distanceToobs3x < threshold2 || distanceToobs4x < threshold2)
    //    //{
    //    //return true;
    //    //}

    //    return false;
    //}

    //private bool TouchesObs()
    //{
    //    // Get the bounds of the base's collider
    //    Collider MobBaseCol = mobileBase.GetComponent<Collider>();
    //    //if (baseCollider == null)
    //    //{
    //    //    Debug.LogError("Base does not have a Collider attached!");
    //    //    return false;
    //    //}

    //    // Array of all obstacles
    //    Collider NavTarCol = tar.GetComponent<Collider>(), 
    //    foreach (var obstacle in obstacles)
    //    {
    //        //if (obstacle == null)
    //        //{
    //        //    Debug.LogError("One or more obstacles are missing colliders!");
    //        //    continue;
    //        //}

    //        // Check if the base's collider intersects with the obstacle's collider
    //        if (baseCollider.bounds.Intersects(obstacle.bounds))
    //        {
    //            return true; // Collision detected
    //        }
    //    }

    //    return false; // No collisions
    //}



}
