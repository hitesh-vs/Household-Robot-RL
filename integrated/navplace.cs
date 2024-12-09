using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class navplace : Agent
{
    public Rigidbody mobileBase;
    public Transform targetPosition;
    public Rigidbody NavTarget;
    public Rigidbody obs1;
    public Rigidbody obs2;
    public Rigidbody obs3;
    public Rigidbody obs4;
    public Rigidbody table;
    public PicknPlace agent1;

    private float previousDistanceToTarget;
    private bool isActive = false;
    public int maxSteps = 500000;

    // Define the bounding box size around the target
    private float boundingBoxSizeX = 5.5f;
    private float boundingBoxSizeZ = 6f;

    //Timekeeping
    private float episodeStartTime; // Time when the episode started
    public float timeLimit = 120f;




    public override void OnEpisodeBegin()
    {
        // Reset Rigidbody velocities
        mobileBase.linearVelocity = Vector3.zero;
        mobileBase.angularVelocity = Vector3.zero;

        // Reset the robot position
        mobileBase.transform.localPosition = new Vector3(2f, 0.05f, 0.05f);
        mobileBase.transform.rotation = Quaternion.Euler(0, 0, 0);

        // Reset the target position
        targetPosition.localPosition = new Vector3(Random.Range(-3f, -1f), 0.1f, Random.Range(-3f, 3f));

        // Randomize obstacles around their initial positions
        List<Vector3> occupiedPositions = new List<Vector3> { targetPosition.localPosition };
        RandomizeObstaclePositionAroundInitial(obs1, new Vector3(1f, 0.1f, 2f), occupiedPositions);
        RandomizeObstaclePositionAroundInitial(obs2, new Vector3(1f, 0.1f, -2f), occupiedPositions);
        RandomizeObstaclePositionAroundInitial(obs3, new Vector3(-0.6f, 0.1f, 1f), occupiedPositions);
        RandomizeObstaclePositionAroundInitial(obs4, new Vector3(-0.6f, 0.1f, -1f), occupiedPositions);

        previousDistanceToTarget = Vector3.Distance(mobileBase.transform.localPosition, targetPosition.localPosition);
        episodeStartTime = Time.time;
    }


    private void RandomizeObstaclePositionAroundInitial(Rigidbody obstacle, Vector3 initialPosition, List<Vector3> occupiedPositions)
    {
        Vector3 newPosition;
        bool isValid;
        float offsetRange = 0.2f; // Adjust the offset range as needed

        do
        {
            // Generate a random position slightly around the initial position
            newPosition = new Vector3(
                initialPosition.x + Random.Range(-offsetRange, offsetRange),
                initialPosition.y, // Keep y fixed
                initialPosition.z + Random.Range(-offsetRange, offsetRange)
            );

            // Check if the position overlaps with any existing positions
            isValid = true;
            foreach (Vector3 occupied in occupiedPositions)
            {
                if (Vector3.Distance(newPosition, occupied) < 0.5f) // Adjust threshold as needed
                {
                    isValid = false;
                    break;
                }
            }
        }
        while (!isValid);

        // Assign the valid position and leave the rotation unchanged
        obstacle.transform.localPosition = newPosition;

        // Add the new position to the list of occupied positions
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
        // Control signal for movement
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.z = actionBuffers.ContinuousActions[1];

        // Move the agent
        float MoveSpeed = 10f;
        mobileBase.position += controlSignal * Time.deltaTime * MoveSpeed;

        // Calculate current distance to target
        float currentDistanceToTarget = Vector3.Distance(mobileBase.transform.localPosition, targetPosition.localPosition);

        if (TouchesTar()) // Goal reached
        {
            SetReward(1.0f); // High reward for reaching the target
            EndEpisode();
            return;
        }

        if (TouchesObs()) // Hit obstacle
        {
            SetReward(-1.0f); // Penalty for hitting obstacles
            EndEpisode();
            return;
        }

        // Reward for progress towards the target
        float progressReward = (previousDistanceToTarget - currentDistanceToTarget);
        if (progressReward > 0)
        {
            AddReward(progressReward * 0.1f); // Adjust scaling factor for progress reward
        }

        // Penalize unnecessary movement
        float distancePenalty = 0.01f * controlSignal.magnitude;
        AddReward(-distancePenalty);

        // Penalize going outside bounding box
        if (IsOutsideBoundingBox())
        {
            AddReward(-1.0f); // Negative reward for leaving the allowed area
            EndEpisode();
            return;
        }

        // Update previous distance
        previousDistanceToTarget = currentDistanceToTarget;

        // Penalize for time exceeding limit
        float timeElapsed = Time.time - episodeStartTime;
        if (timeElapsed >= timeLimit)
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
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

        return false; // The agent is within the bounding box
    }
}
