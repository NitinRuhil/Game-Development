using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Tween : MonoBehaviour
{
    // How much to move on each axis (move from where you start to plus this amount)
    public Vector3 moveAmount = Vector3.zero;

    // Time to take to reach end
    public float smoothTime = 0.2f;

    // This will store where we start
    private Vector3 startingPosition;

    // This will store our target, start position + move amount
    private Vector3 targetPosition;

    // current velocity (start at zero)
    private Vector3 velocity = Vector3.zero;
    private bool movingTowardTarget = true;

    void Start()
    {
        startingPosition = transform.position;
        targetPosition = transform.position + moveAmount;
    }

    void Update()
    {
        if (movingTowardTarget)
        {
            transform.position = Vector3.SmoothDamp(
                transform.position,
                targetPosition,
                ref velocity,
                smoothTime
            );

            if ((transform.position - targetPosition).sqrMagnitude < 0.1f)
            {
                movingTowardTarget = false;
            }
        }
        else
        {
            transform.position = Vector3.SmoothDamp(
                transform.position,
                startingPosition,
                ref velocity,
                smoothTime
            );

            if ((transform.position - startingPosition).sqrMagnitude < 0.1f)
            {
                movingTowardTarget = true;
            }
        }
    }
}