#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

#include <string.h>

struct Move {
	int x;
	int y;
	int z;
};

int read_from_file( char* inputfilename, int board[][4][4] );
int write_to_file( char* outputfilename, int board[4][4][4] );
int write_move_to_file( char* outputfilename, Move m);

int main( int argc, char* argv[] )
{
	char* inputfilename = NULL, *outputfilename = NULL;
	int board[4][4][4];
	Move m = {0,0,0};

	// parse command-line arguments
	for( int i = 1; i < argc; i++ )
	{
		if( !strcmp(argv[i], "-i" ) )
		{
			inputfilename = argv[i+1];
			i++;
		}
		else if( !strcmp( argv[i], "-o") )
		{
			outputfilename = argv[i+1];
			i++;
		}
	}

	// check to make sure command-line arguments have been specified
	if( inputfilename == NULL || outputfilename == NULL )
	{
		printf( "input and output filenames need to be specified on the command line (-i <filename> -o <filename>\n");
		return -1;
	}

	// debug info
	printf( "input file: [%s]\n", inputfilename);
	printf( "output file: [%s]\n", outputfilename);

	// read from file
	read_from_file( inputfilename, board );

	printf( "starting tictactoe\n");

	//sleep(5);

	printf( "finishing tictactoe\n");


	//write board state to file and exit
	return write_move_to_file( outputfilename, m );
}

int read_from_file( char* inputfilename, int board[][4][4] )
{
	FILE *ifile = fopen( inputfilename, "r" );
	if( !ifile )
	{
		printf( "could not open input file [%s] for reading\n", inputfilename );
		return -2;
	}

	for( int i = 0; i < 4; i++ )
	{
		printf( "i: %d\n", i);
		for( int j = 0; j < 4; j++ )
		{
			for( int k = 0; k < 4; k++ )
			{
				char c = '.';
				fscanf(ifile, " %c", &c );
				printf( "i %d j %d k %d\n", i, j, k);
				switch( c )
				{
					case '.': board[i][j][k] = 0; break;
					case 'x': board[i][j][k] = -1; break;
					case 'o': board[i][j][k] = 1; break;
					default: board[i][j][k] = 0; break;
				}

			}
		}
	}

	fclose( ifile );
	return 0;
}

int write_to_file( char* outputfilename, int board[4][4][4] )
{
	FILE *ofile = fopen( outputfilename, "w" );
	if( !ofile )
	{
		printf( "could not open output file [%s] for writing\n", outputfilename );
		return -2;
	}

	// iterate through all squares on the board
	for( int i = 0; i < 4; i++ )
	{
		for( int j = 0; j < 4; j++ )
		{
			for( int k = 0; k < 4; k++ )
			{
				// write the appropriate character to the file
				switch( board[i][j][k])
				{
					case 0: fprintf( ofile, "%c", '.'); break;
					case 1: fprintf( ofile, "%c", 'o'); break;
					case -1: fprintf( ofile, "%c", 'x'); break;
					default: fprintf( ofile, "%c", '.'); break;
				}
			}
			fprintf( ofile, "\n");
		}
		fprintf( ofile, "\n");
	}

	// close the output file
	fclose (ofile);
	return 0;
}

int write_move_to_file( char* outputfilename, Move m)
{
	FILE *ofile = fopen( outputfilename, "w" );
	if( !ofile )
	{
		printf( "could not open output file [%s] for writing\n", outputfilename );
		return -2;
	}
	fprintf( ofile, "%d %d %d\n", m.x, m.y, m.z);
	fclose (ofile);
	return 0;

}

