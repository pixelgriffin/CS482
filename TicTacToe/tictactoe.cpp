#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <string.h>

struct Move {
	int x;
	int y;
	int z;
};

int read_from_file( char* inputfilename, int board[][4][4] );
int write_to_file( char* outputfilename, int board[4][4][4] );
int print_move_to_screen( Move m);

int main( int argc, char* argv[] )
{
	char* inputfilename = NULL, *outputfilename = NULL;
	int board[4][4][4];
	Move m = {0,0,0};
	int ply = 0;

	// parse command-line arguments
	for( int i = 1; i < argc; i++ )
	{
		// input file
		if( !strcmp(argv[i], "-i" ) )
		{
			inputfilename = argv[i+1];
			i++;
		}
		// output file
		else if( !strcmp( argv[i], "-o") )
		{
			outputfilename = argv[i+1];
			i++;
		}
		// number of ply to search ahead
		else if( !strcmp( argv[i], "-p") )
		{
			ply = atoi(argv[i+1]);
			i++;
		}
	}

	// check to make sure command-line arguments have been specified
	if( inputfilename == NULL || outputfilename == NULL )
	{
		printf( "input and output filenames need to be specified on the command line (-i <filename> -o <filename>\n");
		return -1;
	}

	if( ply <= 0 )
	{
		printf( "need to have ply set to be greater than 0 (use -p <ply>)\n");
		return -1;
	}

	// debug info
	//printf( "input file: [%s]\n", inputfilename);
	//printf( "output file: [%s]\n", outputfilename);

	// read from file
	read_from_file( inputfilename, board );

	// debug
	//printf( "starting tictactoe\n");

	//sleep(5);

	//debug into
	//printf( "finishing tictactoe\n");
	
	print_move_to_screen( m );

	//write board state to file and exit
	return write_to_file(outputfilename, board);
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
		for( int j = 0; j < 4; j++ )
		{
			for( int k = 0; k < 4; k++ )
			{
				char c = '.';
				fscanf(ifile, " %c", &c );
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

int print_move_to_screen( Move m)
{
	printf( "%d %d %d\n", m.x, m.y, m.z);
	return 0;

}

