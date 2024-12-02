using UnityEngine;

public class Attach : MonoBehaviour
{
    public bool att = false;  // Variable to track if the object is picked up
    public Vector3 fixedOffset = new Vector3(0, 0, 1);  // Offset of 1 unit along the Z direction

    // Function to activate the following behavior
    public void AttachToEndEffector()
    {
        att = true;  // Set the variable to true when attached
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // If the object is attached, follow the end effector's position and keep fixed orientation
        if (att)
        {
            Transform endEffector = transform.parent;  // Assuming the target is parented to the end effector
            if (endEffector != null)
            {
                // Keep the target 1 unit away from the end effector along the z-axis (no rotation)
                Vector3 targetPosition = endEffector.position + endEffector.forward * fixedOffset.z;
                transform.position = targetPosition;

                // Do not rotate the target, keep its initial orientation
                // Optionally, you can fix it to a specific rotation if you want
                transform.rotation = Quaternion.identity;  // Fix the rotation to a specific orientation (e.g., no rotation)
            }
        }
    }
}
