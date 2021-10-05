using Priority_Queue;
using System.Collections.Generic;
using UnityEngine;

public class Graph : MonoBehaviour
{
    //그래프 리스트
    public List<List<KeyValuePair<UNode, float>>> graph = new List<List<KeyValuePair<UNode, float>>>();

    [SerializeField]
    private UNode[] nodes;//노드 배열

    private UNode endNode;//목적지 노드

    private float[] minDistance = new float[23];//최소비용
    private UNode[] previous = new UNode[23];//경로데이터

    public Stack<UNode> Path { get; set; }//최단 경로

    private const float Infinity = 1000000000;

    private void Awake()
    {
        Path = new Stack<UNode>();
    }

    private void Start()
    {
        //배열 및 리스트 초기화
        for (int i = 1; i < nodes.Length; i++)
        {
            nodes[i].Num = i;
        }

        for (int i = 0; i < 23; i++)
        {
            graph.Add(new List<KeyValuePair<UNode, float>>());
        }

        //거리 무한 초기화
        for (int i = 1; i <= 22; i++)
        {
            minDistance[i] = Infinity;
        }

        Agent agent = FindObjectOfType<Agent>();
        SetEndNode(agent);//목적지 노드 설정

        graph[1].Add(new KeyValuePair<UNode, float>(nodes[2], Vector3.Distance(nodes[1].NodePos, nodes[2].NodePos)));
        graph[1].Add(new KeyValuePair<UNode, float>(nodes[4], Vector3.Distance(nodes[1].NodePos, nodes[4].NodePos)));

        graph[2].Add(new KeyValuePair<UNode, float>(nodes[1], Vector3.Distance(nodes[2].NodePos, nodes[1].NodePos)));
        graph[2].Add(new KeyValuePair<UNode, float>(nodes[13], Vector3.Distance(nodes[2].NodePos, nodes[13].NodePos)));
        graph[2].Add(new KeyValuePair<UNode, float>(nodes[3], Vector3.Distance(nodes[2].NodePos, nodes[3].NodePos)));

        graph[3].Add(new KeyValuePair<UNode, float>(nodes[2], Vector3.Distance(nodes[3].NodePos, nodes[2].NodePos)));
        graph[3].Add(new KeyValuePair<UNode, float>(nodes[4], Vector3.Distance(nodes[3].NodePos, nodes[4].NodePos)));
        graph[3].Add(new KeyValuePair<UNode, float>(nodes[6], Vector3.Distance(nodes[3].NodePos, nodes[6].NodePos)));

        graph[5].Add(new KeyValuePair<UNode, float>(nodes[4], Vector3.Distance(nodes[5].NodePos, nodes[4].NodePos)));
        graph[5].Add(new KeyValuePair<UNode, float>(nodes[6], Vector3.Distance(nodes[5].NodePos, nodes[6].NodePos)));
        graph[5].Add(new KeyValuePair<UNode, float>(nodes[22], Vector3.Distance(nodes[5].NodePos, nodes[22].NodePos)));

        graph[6].Add(new KeyValuePair<UNode, float>(nodes[3], Vector3.Distance(nodes[6].NodePos, nodes[3].NodePos)));
        graph[6].Add(new KeyValuePair<UNode, float>(nodes[5], Vector3.Distance(nodes[6].NodePos, nodes[5].NodePos)));
        graph[6].Add(new KeyValuePair<UNode, float>(nodes[7], Vector3.Distance(nodes[6].NodePos, nodes[7].NodePos)));
        graph[6].Add(new KeyValuePair<UNode, float>(nodes[16], Vector3.Distance(nodes[6].NodePos, nodes[16].NodePos)));

        graph[7].Add(new KeyValuePair<UNode, float>(nodes[6], Vector3.Distance(nodes[7].NodePos, nodes[6].NodePos)));
        graph[7].Add(new KeyValuePair<UNode, float>(nodes[8], Vector3.Distance(nodes[7].NodePos, nodes[8].NodePos)));
        graph[7].Add(new KeyValuePair<UNode, float>(nodes[10], Vector3.Distance(nodes[7].NodePos, nodes[10].NodePos)));
        graph[7].Add(new KeyValuePair<UNode, float>(nodes[12], Vector3.Distance(nodes[7].NodePos, nodes[12].NodePos)));

        graph[8].Add(new KeyValuePair<UNode, float>(nodes[7], Vector3.Distance(nodes[8].NodePos, nodes[7].NodePos)));
        graph[8].Add(new KeyValuePair<UNode, float>(nodes[9], Vector3.Distance(nodes[8].NodePos, nodes[9].NodePos)));

        graph[9].Add(new KeyValuePair<UNode, float>(nodes[8], Vector3.Distance(nodes[9].NodePos, nodes[8].NodePos)));
        graph[9].Add(new KeyValuePair<UNode, float>(nodes[10], Vector3.Distance(nodes[9].NodePos, nodes[10].NodePos)));
        graph[9].Add(new KeyValuePair<UNode, float>(nodes[15], Vector3.Distance(nodes[9].NodePos, nodes[15].NodePos)));

        graph[10].Add(new KeyValuePair<UNode, float>(nodes[7], Vector3.Distance(nodes[10].NodePos, nodes[7].NodePos)));
        graph[10].Add(new KeyValuePair<UNode, float>(nodes[9], Vector3.Distance(nodes[10].NodePos, nodes[9].NodePos)));
        graph[10].Add(new KeyValuePair<UNode, float>(nodes[11], Vector3.Distance(nodes[10].NodePos, nodes[11].NodePos)));

        graph[11].Add(new KeyValuePair<UNode, float>(nodes[10], Vector3.Distance(nodes[11].NodePos, nodes[10].NodePos)));
        graph[11].Add(new KeyValuePair<UNode, float>(nodes[12], Vector3.Distance(nodes[11].NodePos, nodes[12].NodePos)));
        graph[11].Add(new KeyValuePair<UNode, float>(nodes[14], Vector3.Distance(nodes[11].NodePos, nodes[14].NodePos)));

        graph[12].Add(new KeyValuePair<UNode, float>(nodes[7], Vector3.Distance(nodes[12].NodePos, nodes[7].NodePos)));
        graph[12].Add(new KeyValuePair<UNode, float>(nodes[11], Vector3.Distance(nodes[12].NodePos, nodes[11].NodePos)));
        graph[12].Add(new KeyValuePair<UNode, float>(nodes[13], Vector3.Distance(nodes[12].NodePos, nodes[13].NodePos)));

        graph[13].Add(new KeyValuePair<UNode, float>(nodes[2], Vector3.Distance(nodes[13].NodePos, nodes[2].NodePos)));
        graph[13].Add(new KeyValuePair<UNode, float>(nodes[12], Vector3.Distance(nodes[13].NodePos, nodes[12].NodePos)));
        graph[13].Add(new KeyValuePair<UNode, float>(nodes[14], Vector3.Distance(nodes[13].NodePos, nodes[14].NodePos)));

        graph[14].Add(new KeyValuePair<UNode, float>(nodes[11], Vector3.Distance(nodes[14].NodePos, nodes[11].NodePos)));
        graph[14].Add(new KeyValuePair<UNode, float>(nodes[13], Vector3.Distance(nodes[14].NodePos, nodes[13].NodePos)));

        graph[15].Add(new KeyValuePair<UNode, float>(nodes[9], Vector3.Distance(nodes[15].NodePos, nodes[9].NodePos)));
        graph[15].Add(new KeyValuePair<UNode, float>(nodes[17], Vector3.Distance(nodes[15].NodePos, nodes[17].NodePos)));

        graph[16].Add(new KeyValuePair<UNode, float>(nodes[6], Vector3.Distance(nodes[16].NodePos, nodes[6].NodePos)));
        graph[16].Add(new KeyValuePair<UNode, float>(nodes[19], Vector3.Distance(nodes[16].NodePos, nodes[19].NodePos)));
        graph[16].Add(new KeyValuePair<UNode, float>(nodes[20], Vector3.Distance(nodes[16].NodePos, nodes[20].NodePos)));
        graph[16].Add(new KeyValuePair<UNode, float>(nodes[22], Vector3.Distance(nodes[16].NodePos, nodes[22].NodePos)));

        graph[17].Add(new KeyValuePair<UNode, float>(nodes[15], Vector3.Distance(nodes[17].NodePos, nodes[15].NodePos)));
        graph[17].Add(new KeyValuePair<UNode, float>(nodes[18], Vector3.Distance(nodes[17].NodePos, nodes[18].NodePos)));

        graph[18].Add(new KeyValuePair<UNode, float>(nodes[17], Vector3.Distance(nodes[18].NodePos, nodes[17].NodePos)));
        graph[18].Add(new KeyValuePair<UNode, float>(nodes[19], Vector3.Distance(nodes[18].NodePos, nodes[19].NodePos)));
        graph[18].Add(new KeyValuePair<UNode, float>(nodes[20], Vector3.Distance(nodes[18].NodePos, nodes[20].NodePos)));

        graph[19].Add(new KeyValuePair<UNode, float>(nodes[15], Vector3.Distance(nodes[19].NodePos, nodes[15].NodePos)));
        graph[19].Add(new KeyValuePair<UNode, float>(nodes[16], Vector3.Distance(nodes[19].NodePos, nodes[16].NodePos)));
        graph[19].Add(new KeyValuePair<UNode, float>(nodes[18], Vector3.Distance(nodes[19].NodePos, nodes[18].NodePos)));

        graph[20].Add(new KeyValuePair<UNode, float>(nodes[16], Vector3.Distance(nodes[20].NodePos, nodes[16].NodePos)));
        graph[20].Add(new KeyValuePair<UNode, float>(nodes[18], Vector3.Distance(nodes[20].NodePos, nodes[18].NodePos)));
        graph[20].Add(new KeyValuePair<UNode, float>(nodes[19], Vector3.Distance(nodes[20].NodePos, nodes[19].NodePos)));
        graph[20].Add(new KeyValuePair<UNode, float>(nodes[21], Vector3.Distance(nodes[20].NodePos, nodes[21].NodePos)));

        graph[21].Add(new KeyValuePair<UNode, float>(nodes[20], Vector3.Distance(nodes[21].NodePos, nodes[20].NodePos)));
        graph[21].Add(new KeyValuePair<UNode, float>(nodes[22], Vector3.Distance(nodes[21].NodePos, nodes[22].NodePos)));

        graph[22].Add(new KeyValuePair<UNode, float>(nodes[16], Vector3.Distance(nodes[22].NodePos, nodes[16].NodePos)));
        graph[22].Add(new KeyValuePair<UNode, float>(nodes[5], Vector3.Distance(nodes[22].NodePos, nodes[5].NodePos)));
        graph[22].Add(new KeyValuePair<UNode, float>(nodes[21], Vector3.Distance(nodes[22].NodePos, nodes[21].NodePos)));


        agent.SearchNearbyNode();

        Dijkstra(agent.StartNode);
        ShortPath(endNode);

        Debug.Log("최단 비용: " + minDistance[endNode.Num]);
    }

    //목적지와 가장 가까운 노드를 찾는다.
    private void SetEndNode(Agent agent)
    {
        float closeDistance = Infinity;

        for (int i = 1; i < nodes.Length - 1; i++)
        {
            float compareDistance = (nodes[i].transform.position - agent.Destination.transform.position).magnitude;

            if (closeDistance > compareDistance)
            {
                closeDistance = compareDistance;
                endNode = nodes[i];
            }
        }
    }

    private void Dijkstra(UNode start)
    {
        minDistance[start.Num] = 0;

        SimplePriorityQueue<KeyValuePair<UNode, float>> pq = new SimplePriorityQueue<KeyValuePair<UNode, float>>();

        pq.Enqueue(new KeyValuePair<UNode, float>(start, 0f), 0f);

        while (pq.Count != 0)
        {
            UNode current = pq.First.Key;
            float distance = pq.First.Value;

            pq.Dequeue();

            for (int i = 0; i < graph[current.Num].Count; i++)
            {
                UNode next = graph[current.Num][i].Key;
                float nextDistance = distance + graph[current.Num][i].Value;

                if (nextDistance < minDistance[next.Num])
                {
                    minDistance[next.Num] = nextDistance;
                    previous[next.Num] = current;
                    pq.Enqueue(new KeyValuePair<UNode, float>(next, nextDistance), nextDistance);
                }
            }
        }
    }

    //최단 경로 표시
    public void ShortPath(UNode destination)
    {
        Path.Push(destination);

        destination.NodeRenderer.material.SetColor("_Color", Color.red);

        for (int i = previous[destination.Num].Num; i != 0;)
        {
            nodes[i].NodeRenderer.material.SetColor("_Color", Color.red);
            Path.Push(nodes[i]);

            if (previous[i] != null)
            {
                i = previous[i].Num;
            }
            else
            {
                i = 0;
            }
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = new Color(255, 0, 255);
        for (int i = 1; i < graph.Count; i++)
        {
            foreach (var node in graph[i])
            {
                //엣지 그리기
                Gizmos.DrawLine(nodes[i].NodePos, node.Key.NodePos);
            }
        }
    }
}