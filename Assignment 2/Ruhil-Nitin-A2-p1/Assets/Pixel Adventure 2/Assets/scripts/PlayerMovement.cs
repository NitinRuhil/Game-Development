using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class PlayerMovement : MonoBehaviour
{
    private Rigidbody2D rigidbody;
    private Animator animator;
    public Transform jumpCheck;

    bool jumpPressed = false;

    // Start is called before the first frame update
    void Start()
    {
        rigidbody = GetComponent<Rigidbody2D>();
        animator = GetComponent<Animator>();

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        float horizontalInput = Input.GetAxis("Horizontal");
        rigidbody.velocity = new Vector2(horizontalInput * 5.0f, rigidbody.velocity.y);
        animator.SetFloat("Speed", Mathf.Abs(rigidbody.velocity.x));
        if (rigidbody.velocity.x > 0 && transform.localScale.x < 0 || rigidbody.velocity.x < 0 && transform.localScale.x > 0)
        {
            Vector3 newScale = new Vector3(-1f * transform.localScale.x, transform.localScale.y, transform.localScale.z);
            transform.localScale = newScale;


        }
        if (jumpPressed && Physics2D.OverlapCircle(jumpCheck.position, 0.1f, LayerMask.GetMask("Ground")))
        {
            rigidbody.AddForce(new Vector2(0f, 400f));
            jumpPressed = false;
        }

    }
    void Update()
    {
        if (Input.GetButtonDown("Jump"))
        {
            jumpPressed = true;
        }
    }
}
