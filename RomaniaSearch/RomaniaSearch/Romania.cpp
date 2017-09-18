#include <stdio.h>

//The number of vertices stored in our graph
int vertexCount = 0;
//A 2D array of edges in the format graph[vertexA][vertexB] = distance between vertexA and vertexB
int** graph;
//The output logging file. Don't worry about this one
FILE* output;

/*
AddEdge: Add an edge to the graph from vertex u to vertex v.

Arugments:
	int u: One of the vertices that form the edge
	int v: The other vertice that forms the edge
	int dist: The distance between u and v

Adds an edge to the graph from vertex u to vertex v with the given distance dist.
The graph is represented a a 2D array of integers, where each integer is a distance.
When an edge is added from u to v an edge from v to u is also saved in memory.

Returns void: No value returned
*/
void AddEdge(int u, int v, int dist)
{
	graph[u][v] = dist;
	graph[v][u] = dist;
}

/*
ReadGraphFile: Read graph data in from a file to populate our graph structure.

Arguments:
	char* fname: The name of the file to read

Reads graph data in from a file to populate our graph structure. The format of the file
defines a number of vertices at the top of the file, and edges for the graph for each
line afterwards in the format (U, V, DIST).

Returns bool: Returns true if the file was opened and read successfuly, and false if the file could not be read
*/
bool ReadGraphFile(char* fname)
{
	FILE* file = fopen(fname, "r");
	if (file)
	{
		rewind(file);
		int a, b, d;
		
		fscanf(file, "%d", &vertexCount);
		//fscanf_s(file, "%d", &vertexCount);

		graph = new int*[vertexCount];
		for (int u = 0; u < vertexCount; u++)
		{
			graph[u] = new int[vertexCount];

			for (int v = 0; v < vertexCount; v++)
			{
				graph[u][v] = 0;
			}
		}

		while (fscanf(file, "%d %d %d", &a, &b, &d) != EOF)
//		while (fscanf_s(file, "%d %d %d", &a, &b, &d) != EOF)
		{
			AddEdge(a, b, d);
		}

		fclose(file);

		return true;
	}
	else
	{
		printf("Failed to read file...\n");
		return false;
	}
}

/*
LogVertex: Prints the current vertex traveled to and saves the output to a file

Arguments:
	int vert: The vertex that you are currently visiting

In order for us to grade you, you must print the vertex you are travelling to in your algorithm.
This function makes it easier for you to print the proper data. It also saves the data to a file.

Returns void: No values returned
*/
void LogVertex(int vert)
{
	printf("%d\n", vert);
	fprintf(output, "%d\n", vert);
}

/*
SetartLogging: Opens the logging file for later writing

Arguments:
	None

Pretty much just a wrapper for fopen so that we make sure you are writing your logging data to the right file.

Returns void: No values returned
*/
void StartLogging()
{
	output = fopen("log.txt", "w");
}

/*
EndLogging: Closes the logging file properly

Arguments:
	None

Pretty much just a wrapper for fclose so that we make sure you close the right file.

Returns void: No values returned
*/
void EndLogging()
{
	fclose(output);
}

int main(int argc, char** argv)
{
	//Setup
	//-----------------------------------
	if (argc == 3)
	{
		if (!ReadGraphFile(argv[1]))
			return 2;
	}
	else
	{
		printf("No file argument given...\n");
		return 1;
	}
	StartLogging();
	//-------------------------------------



	//Your code


	//Cleanup
	//-------------------------------------
	EndLogging();
	getchar();
	//-------------------------------------

	return 0;
}
