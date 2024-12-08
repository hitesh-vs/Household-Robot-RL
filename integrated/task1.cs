using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents;
using UnityEngine;
using System.Collections;
using static UnityEngine.GraphicsBuffer;
using Unity.Burst.Intrinsics;
using System.Collections.Generic;

public class task1 : Agent
{
    public Rigidbody mobileBase;
    public Transform targetPosition;
    public Rigidbody NavTarget;
    public Rigidbody obs1;
    public Rigidbody obs2;
    public Rigidbody obs3;
    public Rigidbody obs4;
    public Rigidbody table;
    //public PicknPlace agent1;

    public Rigidbody arm1;
    public Rigidbody arm2;
    public Rigidbody endEffector;
    public Rigidbody dummy;

    private Vector3 initialArm1Position = new Vector3(3.5f, 2f, -3.5f);
    private Quaternion initialArm1Rotation = Quaternion.Euler(0f, 0f, 0f);

    private Vector3 initialArm2Position = new Vector3(3.5f, 3.5f, -3.3f);
    private Quaternion initialArm2Rotation = Quaternion.Euler(90f, 0f, 0f);

    private Vector3 initialEndEffectorPosition = new Vector3(3.5f, 3.5f, -3.05f);
    private Quaternion initialEndEffectorRotation = new Quaternion(0, 0, 0, 1);

    private Vector3 finalArm1Position = new Vector3(2f, 0.4f, 0f);
    private Vector3 finalArm2Position = new Vector3(2f, 0.7f, 0.2f);
    private Vector3 finalEndEffectorPosition = new Vector3(2f, 0.7f, 0.45f);

    //private Vector3 initialTargetPosition;
    //private Quaternion initialTargetRotation;


    private float previousDistanceToTarget;
    public bool task1done;
    public int maxSteps = 500000;

    // Define the bounding box size around the target
    private float boundingBoxSizeX = 5.5f;
    private float boundingBoxSizeZ = 6f;

    //Timekeeping
    private float episodeStartTime; // Time when the episode started
    public float timeLimit = 120f;

    void Update()
    {
        if (dummy != null && dummy.gameObject.activeSelf)
        {
            dummy.gameObject.SetActive(false);
            Debug.Log($"Forced deactivation of dummy {dummy.name}.");
        }
    }



    public override void OnEpisodeBegin()
    {
        if (!task1done)
        {

            // Reset arm1
            arm1.transform.position = initialArm1Position;
            arm1.transform.rotation = initialArm1Rotation;

            //Rigidbody arm1Rb = arm1.GetComponent<Rigidbody>();
            if (arm1 != null)
            {
                arm1.linearVelocity = Vector3.zero;
                arm1.angularVelocity = Vector3.zero;
                arm1.useGravity = false; // Disable gravity
                arm1.constraints = RigidbodyConstraints.FreezePositionY; // Freeze necessary axes
            }

            // Reset arm2
            arm2.transform.position = initialArm2Position;
            arm2.transform.rotation = initialArm2Rotation;

            //Rigidbody arm2Rb = arm2.GetComponent<Rigidbody>();
            if (arm2 != null)
            {
                arm2.linearVelocity = Vector3.zero;
                arm2.angularVelocity = Vector3.zero;
                arm2.useGravity = false; // Disable gravity
                arm2.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation; // Freeze necessary axes
            }

            // Reset end-effector
            endEffector.transform.position = initialEndEffectorPosition;
            endEffector.transform.rotation = initialEndEffectorRotation;

            //Rigidbody eeRb = endEffector.GetComponent<Rigidbody>();
            if (endEffector != null)
            {
                endEffector.linearVelocity = Vector3.zero;
                endEffector.angularVelocity = Vector3.zero;
                endEffector.useGravity = false; // Disable gravity
                endEffector.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation; // Freeze necessary axes

            }

            //if (dummy != null)
            //{
            //    dummy.linearVelocity = Vector3.zero;
            //    dummy.angularVelocity = Vector3.zero;
            //    dummy.useGravity = false; // Disable gravity
            //    dummy.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation;

            //}

            // Reset Rigidbody velocities
            mobileBase.linearVelocity = Vector3.zero;
            mobileBase.angularVelocity = Vector3.zero;

            // Find a safe spawn position
            Vector3 spawnPosition;
            spawnPosition = new Vector3(3.5f, 0.05f, -3.5f);

            // Set position and reset orientation
            mobileBase.transform.localPosition = spawnPosition;
            mobileBase.transform.rotation = Quaternion.Euler(0, 0, 0);

            // Previous distance
            previousDistanceToTarget = Vector3.Distance(mobileBase.transform.localPosition, targetPosition.localPosition);

            // Randomize target position within a 5x5 area
            targetPosition.localPosition = new Vector3(2f, 0.05f, 0.05f);
            ////obs 
            //obs1.transform.localPosition = new Vector3(1f, 0.1f, 2f);
            //obs2.transform.localPosition = new Vector3(1f, 0.1f, -2f);
            //obs3.transform.localPosition = new Vector3(-0.6f, 0.1f, 1f);
            //obs4.transform.localPosition = new Vector3(-0.6f, 0.1f, -1f);

            List<Vector3> occupiedPositions = new List<Vector3> { targetPosition.localPosition };
            RandomizeObstaclePosition(obs1, occupiedPositions);
            RandomizeObstaclePosition(obs2, occupiedPositions);
            RandomizeObstaclePosition(obs3, occupiedPositions);
            RandomizeObstaclePosition(obs4, occupiedPositions);

            // Track the previous distance
            previousDistanceToTarget = Vector3.Distance(mobileBase.transform.localPosition, targetPosition.localPosition);


            episodeStartTime = Time.time;
        }


        if (task1done)
        {
            if (arm1 != null)
            {
                arm1.constraints = RigidbodyConstraints.None; // Remove all constraints
            }

            if (arm2 != null)
            {
                arm2.constraints = RigidbodyConstraints.None; // Remove all constraints
            }

            if (endEffector != null)
            {
                endEffector.constraints = RigidbodyConstraints.None; // Remove all constraints
            }

            mobileBase.transform.localPosition = new Vector3(2.0f, 0.05f, 0f);
            NavTarget.gameObject.SetActive(false);
            this.enabled = false; // Prevents further execution


            //NavTarget.SetActive(false);
            Debug.Log("First Nav disabled.");
        }
    }

    private void RandomizeObstaclePosition(Rigidbody obstacle, List<Vector3> occupiedPositions)
    {
        Vector3 newPosition;
        bool isValid;

        do
        {
            // Generate a random position within the range
            newPosition = new Vector3(
                Random.Range(-4f, 1f),
                0.1f,
                Random.Range(-4f, 4f)
            );

            // Check if the position overlaps with any existing positions
            isValid = true;
            foreach (Vector3 occupied in occupiedPositions)
            {
                if (Vector3.Distance(newPosition, occupied) < 0.5f) // Adjust the threshold as needed
                {
                    isValid = false;
                    break;
                }
            }
        }
        while (!isValid);

        // Assign the valid position and add it to the list of occupied positions
        obstacle.transform.localPosition = newPosition;
        occupiedPositions.Add(newPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float baseX = mobileBase.transform.localPosition.x;
        //float baseY = mobileBase.transform.localPosition.y;
        float baseZ = mobileBase.transform.localPosition.z;
        float targetX = targetPosition.localPosition.x;
        float targetY = targetPosition.localPosition.y;
        float targetZ = targetPosition.localPosition.z;

        // Add observations
        sensor.AddObservation(baseX);
        //sensor.AddObservation(baseY);
        sensor.AddObservation(baseZ);
        sensor.AddObservation(targetX);
        //sensor.AddObservation(targetY);
        sensor.AddObservation(targetZ);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //if (!isActive)
        //{
        //    Debug.Log("nav still inactive");
        //}// Skip actions if agent2 is not active

        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Arm1");  // Control base movement (x)
        continuousActionsOut[1] = Input.GetAxis("Arm2");    // Control base movement (z)
        //continuousActionsOut[2] = Input.GetAxis("Arm1");        // Control arm1 joint angle
        //continuousActionsOut[3] = Input.GetAxis("Arm2");        // Control arm2 joint angle
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {

        //if (!isActive)
        //{
        //    // Check if agent1 has reached its target
        //    if (agent1 != null && agent1.GetComponent<PicknPlace>().targetReached)
        //    {
        //        Debug.Log("Agent2 Activated!");
        //        StartCoroutine(ActivateAgentWithDelay());
        //    }
        //    return;

        //    // Coroutine to delay activation
        //    IEnumerator ActivateAgentWithDelay()
        //    {
        //        yield return new WaitForSeconds(5); // Wait for 2 seconds
        //        isActive = true;
        //    }// Skip further processing until activation
        //}

        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.z = actionBuffers.ContinuousActions[1];

        // Move the agent based on the action
        float MoveSpeed = 10f;
        mobileBase.position += controlSignal * Time.deltaTime * MoveSpeed;

        // Reward calculations based on distance to target
        float currentDistanceToTarget = Vector3.Distance(mobileBase.transform.localPosition, targetPosition.localPosition);

        arm1.transform.rotation = initialArm1Rotation; // Lock arm1 to its initial rotation
        arm2.transform.rotation = initialArm2Rotation; // Lock arm2 to its initial rotation

        if (TouchesTar()) // Goal reached
        {
            task1done = true;
            SetReward(1.0f); // High reward for reaching the target
            Debug.Log("Task1 target reached");
            EndEpisode();
        }

        if (TouchesObs()) // Goal reached
        {
            SetReward(-1.0f); // High reward for reaching the target
            Debug.Log("Touched obs");
            EndEpisode();
        }
        //else
        //{
        float rewardForProgress = (previousDistanceToTarget - currentDistanceToTarget);
        if (rewardForProgress >= 0.1)
        {

            SetReward(0.01f); // Reward for progress
        }
        //}

        //Penalize for going outside the bounding box
        if (IsOutsideBoundingBox())
        {
            AddReward(-1.0f); // Negative reward for leaving the allowed area
            Debug.Log("Outside Bb");
            EndEpisode(); // End episode if agent leaves the bounding box
        }

        //if (mobileBase.transform.localPosition.y < 0.0f)
        //{
        //    //Debug.Log("Eps end due to base fall");
        //    SetReward(-1.0f);
        //    EndEpisode();

        //}

        previousDistanceToTarget = currentDistanceToTarget;

        float timeElapsed = Time.time - episodeStartTime;

        // Check if the time limit has been exceeded
        if (timeElapsed >= timeLimit)
        {
            // Apply a penalty and end the episode if time limit exceeded
            AddReward(-1.0f);  // Negative reward for exceeding time limit
            EndEpisode();      // End the episode
        }
    }



    private bool TouchesTar()
    {
        Collider MobBaseCol = mobileBase.GetComponent<Collider>();
        Collider NavTarCol = NavTarget.GetComponent<Collider>();

        // Check if the base's collider intersects with the target's collider
        if (MobBaseCol.bounds.Intersects(NavTarCol.bounds))
        {
            return true; // Collision detected
        }

        return false; // No collision
    }

    private bool TouchesObs()
    {
        Collider mobBaseCol = mobileBase.GetComponent<Collider>();
        GameObject[] obstacles = GameObject.FindGameObjectsWithTag("Obstacle");
        Collider TableCol = table.GetComponent<Collider>();

        foreach (GameObject obs in obstacles)
        {
            Collider obsCol = obs.GetComponent<Collider>();
            if (mobBaseCol.bounds.Intersects(obsCol.bounds))
            {
                return true; // Collision detected with at least one obstacle
            }
        }

        if (mobBaseCol.bounds.Intersects(TableCol.bounds))
        {
            return true; // Collision detected with at least one obstacle
        }


        return false; // No collisions detected
    }


    private bool IsOutsideBoundingBox()
    {
        // Get the position of the agent relative to the target
        Vector3 agentPosition = mobileBase.transform.localPosition;

        // Check if the agent is outside the allowed bounding box
        if (Mathf.Abs(agentPosition.x - targetPosition.localPosition.x) > boundingBoxSizeX ||
            Mathf.Abs(agentPosition.z - targetPosition.localPosition.z) > boundingBoxSizeZ)
        {
            return true; // The agent is outside the bounding box
        }

        return false; // The agent is within the bounding box
    }
}
