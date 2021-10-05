using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UNode : MonoBehaviour
{
    private int num;

    public int Num
    {
        get
        {
            return num;
        }
        set
        {
            num = value;
        }
    }

    public Renderer NodeRenderer
    {
        get
        {
            return GetComponent<Renderer>();
        }
    }

    public Vector3 NodePos
    {
        get
        {
            return transform.position;
        }
    }

    private void Awake()
    {
        num = 1;
    }
}