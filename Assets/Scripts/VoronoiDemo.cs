using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using System.Collections.Generic;
using Delaunay;
using Delaunay.Geo;

namespace projetIA
{

public class Human : MonoBehaviour
{
	public NavMeshAgent agent;
	public GameObject work;
	public Vector3 work_coor;
	public GameObject home;
	public Vector3 home_coor;

	public Human(NavMeshAgent new_agent, GameObject new_home, Vector3 new_home_coor,GameObject new_work, Vector3 new_work_coor)
	{	
		agent = new_agent;
		home = new_home;
		home_coor = new_home_coor;
		work = new_work;
		work_coor = new_work_coor;
	}

	public void power_off(GameObject obj)
	{
		Transform light = obj.transform.GetChild(1);
		if (light.position.y > 0)
		{
			light.Translate(0, -10, 0);
		}
	}

	public void power_on(GameObject obj)
	{
		Transform light = obj.transform.GetChild(1);
		if (light.position.y < 0)
		{
			light.Translate(0, 10, 0);
		}
	}

	public void go_to_work()
	{
		power_off(home);
		agent.destination = work_coor;
		power_on(work);
	}

	public void come_back_home()
	{
		power_off(work);
		agent.destination = home_coor;
		power_on(home);
	}
}


public class VoronoiDemo : MonoBehaviour
{

    public Material land;
    public const int NPOINTS = 130;
    public const int WIDTH = 2000;
    public const int HEIGHT = 2000;
	public float freqx = 0.02f, freqy = 0.018f, offsetx = 0.43f, offsety = 0.22f;
	public GameObject road;
	public GameObject house;
	public GameObject building;
    public NavMeshSurface surface;
	public NavMeshAgent agent;
	public List<Human> humans;

    private List<Vector2> m_points;
	private List<LineSegment> m_edges = null;
	private List<LineSegment> m_spanningTree;
	private List<LineSegment> m_delaunayTriangulation;
	private Texture2D tx;
	
    public LayerMask m_LayerMask;

	private float [,] createMap() 
    {
        float [,] map = new float[WIDTH, HEIGHT];
        for (int i = 0; i < WIDTH; i++)
            for (int j = 0; j < HEIGHT; j++)
                map[i, j] = Mathf.PerlinNoise(freqx * i + offsetx, freqy * j + offsety);
        return map;
    }

	void Start ()
	{
        float [,] map=createMap();
        Color[] pixels = createPixelMap(map);
		List<GameObject> houses = new List<GameObject> ();
		List<Vector3> houses_coord = new List<Vector3> ();
		List<GameObject> buildings = new List<GameObject> ();
		List<Vector3> buildings_coord = new List<Vector3> ();

        /* Create random points points */
		m_points = new List<Vector2> ();
		List<uint> colors = new List<uint> ();
		for (int i = 0; i < NPOINTS; i++) 
		{
			int x = (int)Random.Range(0, WIDTH - 1);
			int y = (int)Random.Range(0, HEIGHT - 1);
			int iter = 0;
			while (map[x,y] < 0.7 && iter<10)
			{
				x = (int)Random.Range(0, WIDTH - 1);
				y = (int)Random.Range(0, HEIGHT - 1);
				iter++;
			}
			colors.Add ((uint)0);
			Vector2 vec = new Vector2(x,y); 
			m_points.Add (vec);
		}

		/* Generate Graphs */
		Delaunay.Voronoi v = new Delaunay.Voronoi (m_points, colors, new Rect (0, 0, WIDTH, HEIGHT));
		m_edges = v.VoronoiDiagram ();
		m_spanningTree = v.SpanningTree (KruskalType.MINIMUM);
		m_delaunayTriangulation = v.DelaunayTriangulation ();

		/* Shows Voronoi diagram */
		Color color = Color.blue;
		for (int i = 0; i < m_edges.Count; i++) {
			LineSegment seg = m_edges [i];				
			Vector2 left = (Vector2)seg.p0;
			Vector2 right = (Vector2)seg.p1;
			Vector2 segment = (right - left)/WIDTH*100;
			float a = Vector2.SignedAngle(Vector2.right, right - left);
			DrawLine (pixels,left, right,color);
			GameObject go_road = Instantiate(road, new Vector3(left.y/WIDTH*100 - 50, 0,left.x/HEIGHT*100 - 50), Quaternion.Euler(0,a+90,0));
			go_road.transform.localScale = new Vector3(segment.magnitude, 1, 1);
			
			float new_x = (left.x + right.x)/2;
			float new_y = (left.y + right.y)/2;		
			if (map[(int)(Mathf.Floor(new_x)) , (int)(Mathf.Floor(new_y))] < 0.7)
			{
				Vector3 house_loc = new Vector3(new_y/WIDTH*100 - 50 , 0, new_x/HEIGHT*100 - 50);
				GameObject go_house = Instantiate(house, house_loc , Quaternion.Euler(0,a+90,0));
				bool collision = MyCollisions(go_house);
				if (collision==false)
				{
					houses.Add(go_house);
					houses_coord.Add(house_loc);
				}
			}
			else
			{
				Vector3 building_loc = new Vector3(new_y/WIDTH*100 - 50 , 0, new_x/HEIGHT*100 - 50);
				GameObject go_building = Instantiate(building, building_loc , Quaternion.Euler(0,a+90,0));
				bool collision = MyCollisions(go_building);
				if (collision==false)
				{
					buildings.Add(go_building);
					buildings_coord.Add(building_loc);
				}
			}
		}

		/* Apply pixels to texture */
		tx = new Texture2D(WIDTH, HEIGHT);
        land.SetTexture ("_MainTex", tx);
		tx.SetPixels (pixels);
		tx.Apply ();

		
        surface.BuildNavMesh();

		
		/* Generate humans */
		for (int i=0; i < houses_coord.Count; i++)
		{
			NavMeshAgent ag = (NavMeshAgent)Instantiate(agent, houses_coord[i], Quaternion.identity);
			int i_rand = Random.Range(0, buildings_coord.Count);
			humans.Add(new Human(ag , houses[i], houses_coord[i], buildings[i_rand], buildings_coord[i_rand]));
		}
		foreach (Human human in humans)
		{
			human.power_off(human.work);
		}

		
		/* Simulate one day */
		StartCoroutine(morning(humans));
		StartCoroutine(evening(humans));
	}

	IEnumerator morning(List<Human> humans)
    {
        Debug.Log("Started Coroutine at timestamp : " + Time.time);
        yield return new WaitForSeconds(3);
        Debug.Log("Finished Coroutine at timestamp : " + Time.time);

		foreach (Human human in humans)
		{
			human.go_to_work();
		}
    }

	IEnumerator evening(List<Human> humans)
    {
        Debug.Log("Started Coroutine at timestamp : " + Time.time);
        yield return new WaitForSeconds(40);
        Debug.Log("Finished Coroutine at timestamp : " + Time.time);

		foreach (Human human in humans)
		{
			human.come_back_home();
		}
    }

	bool MyCollisions(GameObject obj)
    {
        Collider[] hitColliders = Physics.OverlapBox(obj.transform.position, transform.localScale / 2, Quaternion.identity, m_LayerMask);
        if ( 3 < hitColliders.Length)
        {
            obj.transform.Translate(0, -20, 0);
			return true;
        }
		return false;
    }

    /* Functions to create and draw on a pixel array */
    private Color[] createPixelMap(float[,] map)
    {
        Color[] pixels = new Color[WIDTH * HEIGHT];
        for (int i = 0; i < WIDTH; i++)
            for (int j = 0; j < HEIGHT; j++)
            {
                pixels[i * HEIGHT + j] = Color.Lerp(Color.white, Color.black, map[i, j]);
            }
        return pixels;
    }
    private void DrawPoint (Color [] pixels, Vector2 p, Color c) {
		if (p.x<WIDTH&&p.x>=0&&p.y<HEIGHT&&p.y>=0) 
		    pixels[(int)p.x*HEIGHT+(int)p.y]=c;
	}
	// Bresenham line algorithm
	private void DrawLine(Color [] pixels, Vector2 p0, Vector2 p1, Color c) {
		int x0 = (int)p0.x;
		int y0 = (int)p0.y;
		int x1 = (int)p1.x;
		int y1 = (int)p1.y;

		int dx = Mathf.Abs(x1-x0);
		int dy = Mathf.Abs(y1-y0);
		int sx = x0 < x1 ? 1 : -1;
		int sy = y0 < y1 ? 1 : -1;
		int err = dx-dy;
		while (true) {
            if (x0>=0&&x0<WIDTH&&y0>=0&&y0<HEIGHT)
    			pixels[x0*HEIGHT+y0]=c;

			if (x0 == x1 && y0 == y1) break;
			int e2 = 2*err;
			if (e2 > -dy) {
				err -= dy;
				x0 += sx;
			}
			if (e2 < dx) {
				err += dx;
				y0 += sy;
			}
		}
	}
}
}