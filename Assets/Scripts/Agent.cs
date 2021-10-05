using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class Agent : MonoBehaviour
{
    [SerializeField]
    private float maxSpeed;

    private int deceleration = 2;

    private Vector3 velocity;

    public UNode StartNode { get; set; }

    [SerializeField]
    private GameObject destination;
    public GameObject Destination
    {
        get
        {
            return destination;
        }
    }

    private UNode next;

    private Graph graph;

    private bool isSeek;
    private bool isFollowPath;
    private bool isArrive;

    private Rigidbody rb;

    private void Awake()
    {
        deceleration = 1;

        isSeek = false;
        isFollowPath = false;
        isArrive = false;

        rb = GetComponent<Rigidbody>();
    }

    private void Start()
    {
        graph = FindObjectOfType<Graph>();
    }

    private void FixedUpdate()
    {
        if (isSeek)
        {
            //시작 노드까지 이동
            velocity = velocity + Seek(StartNode.transform.position) * Time.deltaTime;
        }

        if (isFollowPath)
        {
            float speed = maxSpeed * 20;

            //다음 노드로 이동
            transform.position = Vector3.MoveTowards(transform.position, next.transform.position, speed * Time.deltaTime);

            //현재 위치가 다음 노드 위치이고, 경로가 남아있으면
            if (transform.position == next.transform.position && graph.Path.Count != 0)
            {
                Debug.LogError(next);
                next = graph.Path.Pop();
            }
            //현재 위치가 다음 노드 위치이고, 경로가 남아있지 않으면
            if (transform.position == next.transform.position && graph.Path.Count == 0)
            {
                isFollowPath = false;
                isArrive = true;
            }
        }

        if (isArrive)
        {
            //마지막 노드에서 목적지까지 이동
            velocity = velocity + Arrive(destination.transform.position);

            if (transform.position == destination.transform.position)
            {
                Debug.Log("Destination Reached");
                velocity = Vector3.zero;
                isArrive = false;
            }
        }

        transform.position = transform.position + velocity;
    }

    private void OnTriggerEnter(Collider collider)
    {
        if (StartNode.gameObject == collider.gameObject)
        {
            velocity = Vector3.zero;
            next = graph.Path.Pop();
            Debug.Log(next);
            isSeek = false;
            isFollowPath = true;
        }
    }

    public void SearchNearbyNode()
    {
        UNode[] nodes = FindObjectsOfType<UNode>();

        float minDistance = 1000000000f;
        float compareDistance;

        foreach (var node in nodes)
        {
            compareDistance = (transform.position - node.transform.position).magnitude;

            if (minDistance > compareDistance)
            {
                minDistance = compareDistance;
                StartNode = node;
            }
        }

        isSeek = true;
    }

    private Vector3 Seek(Vector3 targetPos)
    {
        var desiredVelocity = (targetPos - transform.position).normalized * maxSpeed;

        desiredVelocity.y = 0.0f;

        return (desiredVelocity - velocity);
    }

    private Vector3 Arrive(Vector3 targetPos)
    {
        float distance = Vector3.Distance(targetPos, transform.position);

        if (distance > 0.0f)
        {
            Vector3 to_target = targetPos - transform.position;

            float _speed = distance / deceleration;

            // 최대 속도로 제한.
            _speed = Mathf.Min(_speed, maxSpeed);

            Vector3 desired_velocity = to_target / distance * _speed;

            return (desired_velocity - velocity);
        }

        return Vector3.zero;
    }
}﻿