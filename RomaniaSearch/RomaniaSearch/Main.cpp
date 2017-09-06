#include <stdio.h>
#include <vector>

using namespace std;

int vertexCount = 0;
int** graph;

void AddEdge(int u, int v, int dist)
{
	graph[u][v] = dist;
	graph[v][u] = dist;
}

void ReadGraphFile(char* fname)
{
	FILE* file = fopen(fname, "r");
	if (file)
	{
		rewind(file);
		int a, b, d;

		fscanf_s(file, "%d", &vertexCount);

		graph = new int*[vertexCount];
		for (int u = 0; u < vertexCount; u++)
		{
			graph[u] = new int[vertexCount];

			for (int v = 0; v < vertexCount; v++)
			{
				graph[u][v] = 0;
			}
		}

		while (fscanf_s(file, "%d %d %d", &a, &b, &d) != EOF)
		{
			AddEdge(a, b, d);
		}

		fclose(file);
	}
	else
	{
		printf("Failed to read file...\n");
	}
}

//Call this once when you visit a new node
void PrintVertex(int vert)
{
	printf("%d\n", vert);
}

int main(int argc, char** argv)
{
	ReadGraphFile("graph.txt");

	//...

	getchar();

	return 0;
}