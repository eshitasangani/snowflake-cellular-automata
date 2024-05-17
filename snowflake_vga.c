///////////////////////////////////////
/// 640x480 version!
/// change to fixed point 
/// compile with:
/// gcc snowflake_vga.c -o snow
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 

/* Cyclone V FPGA devices */
#define HW_REGS_BASE          0xff200000
//#define HW_REGS_SPAN        0x00200000 
#define HW_REGS_SPAN          0x00005000                                      

#define FPGA_ONCHIP_BASE      0xC8000000
//#define FPGA_ONCHIP_END       0xC803FFFF
// modified for 640x480
// #define FPGA_ONCHIP_SPAN      0x00040000
#define FPGA_ONCHIP_SPAN      0x00080000

#define FPGA_CHAR_BASE        0xC9000000 
#define FPGA_CHAR_END         0xC9001FFF
#define FPGA_CHAR_SPAN        0x00002000

/* function prototypes */
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_disc (int, int, int, short);
void VGA_cell (int, int, int, int, short);
// fixed pt
typedef signed int fix28 ;
//multiply two fixed 4:28
#define multfix28(a,b) ((fix28)(((( signed long long)(a))*(( signed long long)(b)))>>28)) 
//#define multfix28(a,b) ((fix28)((( ((short)((a)>>17)) * ((short)((b)>>17)) )))) 
#define float2fix28(a) ((fix28)((a)*268435456.0f)) // 2^28
#define fix2float28(a) ((float)(a)/268435456.0f) 
#define int2fix28(a) ((a)<<28);
// the fixed point value 4
#define FOURfix28 0x40000000 
#define SIXTEENTHfix28 0x01000000
#define ONEfix28 0x10000000

// the light weight buss base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

// shared memory 
key_t mem_key=0xf0;
int shared_mem_id; 
int *shared_ptr;
int shared_time;
int shared_note;
char shared_str[64];

// loop identifiers
int i,j,k,x,y;

///////////////////////////////////////////////// 
#define WIDTH 201
#define HEIGHT 201
#define ALPHA 1.0
#define BETA 0.8
#define GAMMA 0.01
#define NUM_NEIGHBORS 6

typedef struct {
    float u;   // water used in diffusion
    float v;   // water not used in diffusion
    float s;   // total water
	float next_u; // water used next step 
	float next_v; // water not used next step 
    bool is_receptive;
} Cell;

Cell cells[WIDTH][HEIGHT];
float s_vals[WIDTH][HEIGHT]; // Array to store s values for visualization or debugging
Cell* neighbors[NUM_NEIGHBORS];
Cell frozen[WIDTH][HEIGHT];
//int frozen_cells[WIDTH][HEIGHT];
int num_neighbors;
// 8-bit color
#define rgb(r,g,b) ((((r)&7)<<5) | (((g)&7)<<2) | (((b)&3)))

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
	char  *pixel_ptr ;\
	pixel_ptr = (char *)vga_pixel_ptr + ((y)<<10) + (x) ;\
	*(char *)pixel_ptr = (color);\
} while(0)


// Get neighbors for a specific coordinate
int get_num_neighbors(Cell* neighbors[], int x, int y) {
	// SET EDGE CELLS TO BE STATIC ??
    int count = 0;

	if (y != 0) { // top neighbor, does not rely on if the column is even or odd
		neighbors[count++] = &cells[x][y-1];
	}
	if (y != HEIGHT-1) { // bottom  neighbor, does not rely on if the column is even or odd
		neighbors[count++] = &cells[x][y+1];
	}
	
	if (x % 2 == 0) { // even columns
		if (x != 0) { // left side neighbors, the 0th column does not have left side neighbors
			neighbors[count++] = &cells[x-1][y];
			if (y != 0) { // only top left if y not 0
				neighbors[count++] = &cells[x-1][y-1];
			}
		}
		if (x != WIDTH-1) { // right side neighbors
			neighbors[count++] = &cells[x+1][y];
			if (y != 0) { // only top right if y not 0
				neighbors[count++] = &cells[x+1][y-1];
			}
		}
		
	}
	if (x % 2 == 1){ // odd columns
		// odd numbered columns always have left side neighbors
		neighbors[count++] = &cells[x-1][y];
		if (y != HEIGHT-1) { // only bottom left if y not at bottom
			neighbors[count++] = &cells[x-1][y+1];
		}

		if (x != WIDTH-1) { // right side neighbors
			neighbors[count++] = &cells[x+1][y];
			if (y != HEIGHT-1) { // only bottom right if y not height-1
				neighbors[count++] = &cells[x+1][y+1];
			}
		}
	}

    return count;
}

// // "length" is the length of the array.   
// #define each(item, array, length) \
// (typeof(*(array)) *p = (array), (item) = *p; p < &((array)[length]); p++, (item) = *p)


void initialize_grid() {
    for ( i = 0; i < WIDTH; i++) {
        for ( j = 0; j < HEIGHT; j++) {
            cells[i][j].s = BETA;
            cells[i][j].is_receptive = false;
            cells[i][j].u = 0;
			cells[i][j].next_u = 0;
			cells[i][j].next_v = 0;
            cells[i][j].v = 0;
        }
    }
    // Set the center cell
    cells[(WIDTH -1 ) / 2 ][(HEIGHT -1 ) / 2].s = 1.0;
    cells[(WIDTH -1 ) / 2][(HEIGHT -1 ) / 2].is_receptive = true;

	// num_neighbors = get_num_neighbors(neighbors, 25, 25);

	// for (k = 0; k < num_neighbors; k++) {
	// 	neighbors[k]->is_receptive = true;
	// 	// printf("%.2f", neighbors[k]->s);
	// 	// printf("s value");
	// 	// printf("\n");
	// }

}

void update_s_vals() {
    for (i = 0; i < WIDTH; i++) {
        for (j = 0; j < HEIGHT; j++) {
            s_vals[i][j] = cells[i][j].s;
        }
    }
}

void print_s_vals() {
    for (i = 0; i < WIDTH; i++) {
        for (j = 0; j < HEIGHT; j++) {
            printf("%.2f ", cells[i][j].s);
        }
        printf("\n");
    }
    printf("\n");
}


float u_avg = 0.0;
float sum_u = 0.0;
short color; 

void one_iter() {

    // step 1: determine receptive sites
    for ( i = 0; i < WIDTH; i++) {
        for ( j = 0; j < HEIGHT; j++) {

            if (cells[i][j].is_receptive) {
                cells[i][j].u = 0;
                cells[i][j].v = cells[i][j].s;
                cells[i][j].next_v = cells[i][j].s + GAMMA;
            } 
			else {
                cells[i][j].u = cells[i][j].s;
                cells[i][j].v = 0;
            }
        }
    }

    // step 2: modify cell values == doing actual diffusion
	for (i = 0; i < WIDTH; i++) {
		for (j = 0; j < HEIGHT; j++) {

			if (i == 0 || i == WIDTH-1 || j == 0 || j == HEIGHT-1) { // for edge cells
				cells[i][j].u = BETA;
				cells[i][j].v = 0;
			}

			else { // everything else
				num_neighbors = get_num_neighbors(neighbors, i, j); // get num neighbors @ specific coord + modify neighbors

				// if (num_neighbors > 0) { // make sure there are neighbors so we can avoid division bt 0
					sum_u = 0;

					for (k = 0; k < num_neighbors; k++) {
						sum_u += neighbors[k]->u; // Sum u values of all neighbors
					}

					u_avg = sum_u / num_neighbors; // Calculate average u
					
					cells[i][j].next_u = cells[i][j].u + (ALPHA / 2 * (u_avg - cells[i][j].u)); // update u based on diffusion eq 
					// printf("%.2f ", cells[i][j].next_u);
					// update the cell
					cells[i][j].s = cells[i][j].next_u + cells[i][j].next_v; 

					// Update receptiveness based on the new sp
					if (cells[i][j].s >= 1) {
						cells[i][j].is_receptive = true;
					}
					// 	//neighbors[k]->is_receptive = true;
					// 	for (k = 0; k < num_neighbors; k++) {
					// 		neighbors[k]->is_receptive = true;
					// 		// printf("%.2f", neighbors[k]->s);
					// 		// printf("s value");
					// 		// printf("\n");

					// 	}
					// } 
			}

			// } 
			// end of if statment checking to make sure if we have neighbors or not 
			// else { // if cell has no neighbors, just make it so that it is not receptive
			// 	cells[i][j].is_receptive = false;
			// }
		}
	}

	// step 3: all s values must be updated to determine new boundary 
	for (i = 0; i < WIDTH; i++) { 
		for (j = 0; j < HEIGHT; j++) { 
			if (!cells[i][j].is_receptive) {
				num_neighbors = get_num_neighbors(neighbors, i, j); 
				// printf("%d", num_neighbors);
				
				for (k = 0; k < num_neighbors; k++) {
					if (neighbors[k]->s >= 1) { 

						// printf("%.2f", neighbors[k]->s);
						// printf("s value");
						// printf("\n");

						// neighbors[k]->is_receptive = true; // this should already be true

						cells[i][j].is_receptive = true; // one of the neighbors is frozen, so we mark this cell as receptive
					} 
					// break;					
				}
				// break;

			}

		}
	}
	//update_s_vals();

 }



void run_snow() {
	// this runs snowflake gen for 1 iteration + updates the cells 
	
	for (i = 0; i < WIDTH; i++) { 
		for (j = 0; j < HEIGHT; j++){
			// if (s_vals[i][j] >= 1) {
			// 	frozen_cells[i][j] = 1;
			// }

			color = cells[i][j].s >= 1 ? rgb(7,7,7) : rgb(0,0,0); 

			// for (x = 0; x < 320; x++) { // column number (x)
			// 	for (y = 0; y < 240; y++ ) {  // row number (y)
					// inside vga now 

			// if even column 
			if (i % 2 == 0) { 
				VGA_box(2*i, 2*j, 2*i + 2, 2*j + 2,  color);
				// float b = 2*i;
				// float c = 2*j;
				// float d = 2*i+2;
				// float e = 2*j+2;

				// printf("%.2f", b);
				// printf(" ");

				// printf("%.2f", c);
				// printf(" ");
				// printf("%.2f", d);
				// printf(" ");
				// printf("%.2f", e);
				// printf("\n");

			}
			else { 
				VGA_box(2*i, 2*j + 1, 2*i + 2, 2*j + 3,  color);
			}
					 
			// 	}
			// }

		}
	}
	// for (i = 0; i < WIDTH; i++) {
	// 	for (j = 0; j < HEIGHT; j++) {

	// 		color = frozen_cells[i][j] == 1 ? rgb(7,7,7) : rgb(0,0,0); 
	// 		// if even column 
	// 		if (i % 2 == 0) { 
	// 			VGA_box(2*i, 2*j, 2*i + 2, 2*j + 2,  color);

	// 		}
	// 		else { 
	// 			VGA_box(2*i, 2*j + 1, 2*i + 2, 2*j + 3,  color);
	// 		}
	// 	}
	// }
}

void draw_VGA_test(){
	// index by the cell numbers
	for (i = 1; i < 640; i++) {  // column number (x)
		for (j = 1; j < 480; j++ ) { // row number (y)
			// void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
			if (j % 2 == 0) {
				// VGA_PIXEL(2*i, 2*(j-1), 2*(i+2), 2*(j-3), 0x1d);
				VGA_box(i-1, j-1, i+1, j+1, 0x1d);
			}  
			else{
				VGA_box(i, j, i-1, j-1, 0x1b);
			}
		}
	}
}


int main(void)
{
	//int x1, y1, x2, y2;

	// Declare volatile pointers to I/O registers (volatile 	// means that IO load and store instructions will be used 	// to access these pointer locations, 
	// instead of regular memory loads and stores) 

	// === shared memory =======================
	// with video process
	shared_mem_id = shmget(mem_key, 100, IPC_CREAT | 0666);
 	//shared_mem_id = shmget(mem_key, 100, 0666);
	shared_ptr = shmat(shared_mem_id, NULL, 0);

  	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
		return(1);
	}
    

	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	vga_pixel_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, 			FPGA_ONCHIP_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================

	/* create a message to be displayed on the VGA 
          and LCD displays */
	char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
	char text_bottom_row[40] = "Cornell ece5760\0";

	//VGA_text (34, 1, text_top_row);
	//VGA_text (34, 2, text_bottom_row);
	// clear the screen
	VGA_box (0, 0, 639, 479, 0x1c);
	// VGA_box (0, 0, 639, 479, 0x1a);
	initialize_grid();

	// while (1) { 

	for (x = 0; x < 145; x++) { 
		one_iter();
	// 	sleep(1);
	//	update_s_vals();
		// print_s_vals();
		run_snow();
	
	}

	// VGA_cell(2*4, 2*4, 2*4 + 2, 2*4 + 2,  0x00);
	// draw_VGA_test();
		

	// }
	
	

	// update_s_vals();
	// print_u_vals();
	// printf("%f", cells[25][25].u);
	// printf("%f", cells[25][25].v);
	// printf("%f", cells[25][25].s);
	// clear the text
	// VGA_text_clear();

    // VGA_text (10, 1, text_top_row);
    // VGA_text (10, 2, text_bottom_row);
    
	//} // end while(1)
} // end main


void VGA_cell(int x1, int y1, int x2, int y2, short pixel_color)
{
	VGA_PIXEL(x1,y1,pixel_color);
	VGA_PIXEL(x1,y2,pixel_color);
	VGA_PIXEL(x2,y1,pixel_color);
	VGA_PIXEL(x2,y2,pixel_color);

}


/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset;
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear()
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset, x, y;
	for (x=0; x<79; x++){
		for (y=0; y<59; y++){
	/* assume that the text string fits on one line */
			offset = (y << 7) + x;
			// write to the character buffer
			*(character_buffer + offset) = ' ';		
		}
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			*(char *)pixel_ptr = pixel_color;		
		}
}

/****************************************************************************************
 * Draw a filled circle on the VGA monitor 
****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++)
		for (xc = -r; xc <= r; xc++)
		{
			col = xc;
			row = yc;
			// add the r to make the edge smoother
			if(col*col+row*row <= rsqr+r){
				col += x; // add the center point
				row += y; // add the center point
				//check for valid 640x480
				if (col>639) col = 639;
				if (row>479) row = 479;
				if (col<0) col = 0;
				if (row<0) row = 0;
				pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
				// set pixel color
				*(char *)pixel_ptr = pixel_color;
			}
					
		}
}

// =============================================
// === Draw a line
// =============================================
//plot a line 
//at x1,y1 to x2,y2 with color 
//Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
	int e;
	signed int dx,dy,j, temp;
	signed int s1,s2, xchange;
     signed int x,y;
	char *pixel_ptr ;
	
	/* check and fix line coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
        
	x = x1;
	y = y1;
	
	//take absolute value
	if (x2 < x1) {
		dx = x1 - x2;
		s1 = -1;
	}

	else if (x2 == x1) {
		dx = 0;
		s1 = 0;
	}

	else {
		dx = x2 - x1;
		s1 = 1;
	}

	if (y2 < y1) {
		dy = y1 - y2;
		s2 = -1;
	}

	else if (y2 == y1) {
		dy = 0;
		s2 = 0;
	}

	else {
		dy = y2 - y1;
		s2 = 1;
	}

	xchange = 0;   

	if (dy>dx) {
		temp = dx;
		dx = dy;
		dy = temp;
		xchange = 1;
	} 

	e = ((int)dy<<1) - dx;  
	 
	for (j=0; j<=dx; j++) {
		//video_pt(x,y,c); //640x480
		pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x; 
		// set pixel color
		*(char *)pixel_ptr = c;	
		 
		if (e>=0) {
			if (xchange==1) x = x + s1;
			else y = y + s2;
			e = e - ((int)dx<<1);
		}

		if (xchange==1) y = y + s2;
		else x = x + s1;

		e = e + ((int)dy<<1);
	}
}