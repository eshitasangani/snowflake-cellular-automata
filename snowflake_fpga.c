///////////////////////////////////////
/// 640x480 version!
/// change to fixed point 
/// compile with:
/// gcc snow.c -o snow -pthread
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
#include <pthread.h>


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

//// BASE ADDRESSES FOR PIO ADDRESSES ////
#define PIO_ALPHA_BASE      0x100
#define PIO_BETA_BASE       0x110
#define PIO_GAMMA_BASE      0x120
#define PIO_RESET_FROM_BASE 0x130
#define PIO_IS_FROZEN_BASE  0x140
#define PIO_FROZEN_Y_BASE   0x150
#define PIO_RESET_TO_BASE   0x160
#define PIO_DONE_SEND_BASE  0x170

// =============================== FPGA ============================
volatile unsigned int *pio_alpha_addr       = NULL;
volatile unsigned int *pio_beta_addr        = NULL;
volatile unsigned int *pio_gamma_addr       = NULL;
volatile unsigned int *pio_reset_addr       = NULL;
volatile unsigned int *pio_is_frozen_addr       = NULL;
// volatile unsigned int *pio_frozen_x_addr    = NULL;
volatile unsigned int *pio_frozen_y_addr    = NULL;
volatile unsigned int *pio_reset_to_addr    = NULL;
volatile unsigned int *pio_done_send_addr   = NULL;


// fix18 frozen_x;
// fix18 frozen_y;

// Variables to store outputs from the FPGA
typedef struct {
    // fix18 frozen_x;
    int is_frozen;
    int frozen_y;
} yCoordinate;

#define BUFFER_SIZE 11
yCoordinate buffer[BUFFER_SIZE];
int buffer_index = 0;

// BUFFER LOCK VARIABLES
pthread_mutex_t buffer_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t buffer_not_empty = PTHREAD_COND_INITIALIZER;
pthread_cond_t buffer_not_full = PTHREAD_COND_INITIALIZER;
pthread_cond_t one_iteration = PTHREAD_COND_INITIALIZER;

/* function prototypes */
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_cell (int, int, int, int, short);
void VGA_rect (int, int, int, int, short);

// MACROS FOR FIXED POINT CONVERSION // 
typedef signed int fix18 ;
#define mult2fix18(a,b) ((fix18)(((( signed long long)(a))*(( signed long long)(b)))>>18)) 
#define float2fix18(a) ((fix18)((a)*262144.0f)) // 2^18
#define fix2float18(a) ((float)(a)/262144.0f) 
#define int2fix28(a) ((a)<<18);

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
    char  *pixel_ptr ;\
    pixel_ptr = (char *)vga_pixel_ptr + ((y)<<10) + (x) ;\
    *(char *)pixel_ptr = (color);\
} while(0)

// 16-bit primary colors
#define red  (0+(0<<5)+(31<<11))
#define dark_red (0+(0<<5)+(15<<11))
#define green (0+(63<<5)+(0<<11))
#define dark_green (0+(31<<5)+(0<<11))
#define blue (31+(0<<5)+(0<<11))
#define dark_blue (15+(0<<5)+(0<<11))
#define yellow (0+(63<<5)+(31<<11))
#define cyan (31+(63<<5)+(0<<11))
#define magenta (31+(0<<5)+(31<<11))
#define black (0x0000)
#define gray (15+(31<<5)+(51<<11))
#define white (0xffff)
int colors[] = {red, dark_red, green, dark_green, blue, dark_blue, 
        yellow, cyan, magenta, gray, black, white};

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

// BUFFER INIT

// char frozen_x_buffer[64];
char is_frozen_buffer[64];
char frozen_y_buffer[64];

// loop identifiers
int i,j,k,x,y;

///////////////////////////////////////////////////////////////
// THREADS ////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

// INITAL VARIABLES TO SEND TO FGPA 
float temp_alpha = 1.0;
float temp_beta = 0.8;
float temp_gamma = 0.01;

// change reset value via trigger 
int init_reset = 0; 
int set = 0;
short color;

/// MUTE

///////////////////////////////////////////////////////////////
// for debug  // 
///////////////////////////////////////////////////////////////
#include <stdio.h>

void print_buffer() {
    // pthread_mutex_lock(&buffer_mutex);  // Lock the buffer for safe access

    printf("buffer content:\n");
    for ( i = 0; i < buffer_index; ++i) {  // Only iterate up to the current buffer index
        printf("idx %d: is_frozen = %d, frozen_y = %d\n", i, buffer[i].is_frozen, buffer[i].frozen_y);
    }

    // pthread_mutex_unlock(&buffer_mutex);  // Unlock the buffer after done reading
}

void * debug_thread() {
    while (1) {
        print_buffer();
        sleep(1);  // Print the buffer contents every second
    }
}

///////////////////////////////////////////////////////////////
// reset thread  // 
///////////////////////////////////////////////////////////////


void clear_buffer() {
    for ( i = 0; i < 11; i++) {  
        buffer[i].frozen_y = 0;
        buffer[i].is_frozen = 0;
    }
    buffer_index = 0;
}


// void * reset_thread() {

//   while (1) {
  
//     if (*pio_reset_to_addr) {
    
//         // update pio pointers w the initial values 
//         *(pio_alpha_addr) = float2fix18(temp_alpha);
//         *(pio_beta_addr)  = float2fix18(temp_beta);
//         *(pio_gamma_addr) = float2fix18(temp_gamma);
//         *pio_done_send_addr = 0;

        
//       // clear VGA screen 
//       VGA_box (0, 0, 639, 479, 0x0000);
//       clear_buffer();

//     }
//   }
// }

///////////////////////////////////////////////////////////////
// scan thread  // 
///////////////////////////////////////////////////////////////
// void * scan_thread () { 

//     while (1) { 
//         printf("1: alpha, 2: beta 3. gamma \n");
//         scanf("%i", &set);

//         switch (set) {
//             case 1: 
//                 printf("enter alpha: ");
//                 scanf("%f", &temp_alpha);
//                 *pio_alpha_addr = float2fix18(temp_alpha);
//                 break;

//             case 2: 
//                 printf("enter beta: ");
//                 scanf("%f", &temp_beta);
//                 *pio_beta_addr = float2fix18(temp_beta);
//                 break;

//             case 3: 
//                 printf("enter gamma: ");
//                 scanf("%f", &temp_gamma);
//                 *pio_gamma_addr = float2fix18(temp_gamma);
//                 break;
//         }
//     }
// }


/////////////////////////////////////////////////////////////
// frozen thread  // 
/////////////////////////////////////////////////////////////
// this thread is responsible for going through the incoming 
// values and addding it to a buffer 
fix18 y;
fix18 y_froze;

void * frozen_thread () { 

    while (1) { 

        // read values from the FPGA 
        y       = *pio_frozen_y_addr;
        y_froze = *pio_is_frozen_addr;

        // Store values in the buffer
        if (buffer_index < BUFFER_SIZE) {
            buffer[buffer_index].is_frozen = y_froze ;
            buffer[buffer_index].frozen_y = y;


            if (buffer[buffer_index].is_frozen == 1) { 
                VGA_rect(100, 10 * buffer[buffer_index].frozen_y + 100, 
                                100 + 10, 10 * buffer[buffer_index].frozen_y + 110, blue);

            }
            else { 
                VGA_rect(100, 10 * buffer[buffer_index].frozen_y + 100, 
                        100 + 10, 10 * buffer[buffer_index].frozen_y + 110, white);
            }


            buffer_index++;

            *pio_done_send_addr = 1;
            *pio_done_send_addr = 0;
 

        }
        else { 
            // this means that the buffer is full 
            
            buffer_index = 0;
            clear_buffer();

        }

    }

}


// Comparison function for qsort
int compare_y(const void *a, const void *b) {
    const yCoordinate *coord_a = (const yCoordinate *)a;
    const yCoordinate *coord_b = (const yCoordinate *)b;
    return coord_a->frozen_y - coord_b->frozen_y;
}

///////////////////////////////////////////////////////////////
// draw thread  // 
///////////////////////////////////////////////////////////////
void * draw_thread () { 

    int local_index;
    yCoordinate coord;
    int center = 0;
    int offset;

    while (1) { 

        if (*pio_reset_to_addr) {

            // update pio pointers w the initial values 
            *(pio_alpha_addr) = float2fix18(temp_alpha);
            *(pio_beta_addr)  = float2fix18(temp_beta);
            *(pio_gamma_addr) = float2fix18(temp_gamma);

            // clear VGA screen
            VGA_box (0, 0, 639, 479, 0x0000);
            // want the buffer to start from the beginning
            // i think we need these three lines upon reset but adding them fucks it up?? 
            // it makes it so that the buffer contet is just empty upon reset!!! 
            // i think smtg is wrong with reset......
            // clear_buffer();

            // pthread_mutex_lock(&buffer_mutex);
            buffer_index = 0;
            // pthread_mutex_unlock(&buffer_mutex);

        }
        else {
            // no reset, we will begin drawing
            *pio_done_send_addr = 1; // signal to the fpga that we don't want to receive new values, until done drawing this buffer.


            // exclusive access to the buffer
            // pthread_mutex_lock(&buffer_mutex);

            // wait for the buffer to be full and then we want to begin to read 
            // buffer_not_empty is a conditional variable
            while (buffer_index <= 0) {
                pthread_cond_wait(&buffer_not_empty, &buffer_mutex);
            }
            // once the buffer has something, unlock mutex and allow other threads 
            pthread_mutex_unlock(&buffer_mutex);

            // now we are gpmma be readomg through the data 

            // Sort the buffer based on frozen_y coordinates
            qsort(buffer, buffer_index, sizeof(yCoordinate), compare_y);

            // Draw the center cell in white 
            // middle element of the sorted buffer, used as sanity check for debugging
            int center_index = buffer_index / 2;
            yCoordinate center_coord = buffer[center_index];
            VGA_rect(100, 10 * center_coord.frozen_y + 100, 
                    100 + 10, 10 * center_coord.frozen_y + 110, yellow);

            // Draw cells outward from the center
            for (offset = buffer_index / 2; offset > 0; --offset) {
                // calculating indices of each cells to draw on either side of the center
                int index1 = (center_index + offset) % buffer_index;
                int index2 = (center_index - offset) % buffer_index;

                // obtain coordinates from the bufer
                yCoordinate coord1 = buffer[index1];
                yCoordinate coord2 = buffer[index2];

                // since we are drawing 2 indices, check if both are frozen/not frozen

                if (coord1.is_frozen == 1) {
                    VGA_rect(100, 10 * coord1.frozen_y + 100, 
                            100 + 10, 10 * coord1.frozen_y + 110, blue);
                } else {
                    VGA_rect(100, 10 * coord1.frozen_y + 100, 
                            100 + 10, 10 * coord1.frozen_y + 110, white);
                }
                if (coord2.is_frozen == 1) {
                    VGA_rect(100, 10 * coord2.frozen_y + 100, 
                            100 + 10, 10 * coord2.frozen_y + 110, blue);
                } else {
                    VGA_rect(100, 10 * coord2.frozen_y + 100, 
                            100 + 10, 10 * coord2.frozen_y + 110, white);
                }
                buffer_index++;      
            }
            // used for controlling the drawing speed
            usleep(500000); 

            
            // for ( buffer_index = 0; j < BUFFER_SIZE; buffer_index++) {  // Only iterate up to the current buffer index]
            //     yCoordinate coord1 = buffer[buffer_index];
            //     if (coord1.is_frozen == 1) {
            //             VGA_rect(100, 10 * coord1.frozen_y + 100, 
            //                     100 + 10, 10 * coord1.frozen_y + 110, blue);
            //         } else {
            //             VGA_rect(100, 10 * coord1.frozen_y + 100, 
            //                     100 + 10, 10 * coord1.frozen_y + 110, white);
            //         }
            // }

            // usleep(500000); 
            pthread_cond_signal(&buffer_not_full); 
            
        //     if (buffer_index > 0 && buffer_index < 12 ) { 
        //         // There is something in the buffer, let's check! 

        //         // Sort the buffer based on frozen_y coordinates
        //         qsort(buffer, buffer_index, sizeof(yCoordinate), compare_y);

        //         // Draw the center cell in white 
        //         // middle element of the sorted buffer, used as sanity check for debugging
        //         int center_index = buffer_index / 2;
        //         yCoordinate center_coord = buffer[center_index];
        //         VGA_rect(100, 10 * center_coord.frozen_y + 100, 
        //                 100 + 10, 10 * center_coord.frozen_y + 110, yellow);

        //         // Draw cells outward from the center
        //         for (offset = buffer_index / 2; offset > 0; --offset) {
        //             // calculating indices of each cells to draw on either side of the center
        //             int index1 = (center + offset) % buffer_index;
        //             int index2 = (center - offset + buffer_index) % buffer_index;

        //             // obtain coordinates from the bufer
        //             yCoordinate coord1 = buffer[index1];
        //             yCoordinate coord2 = buffer[index2];

        //             // since we are drawing 2 indices, check if both are frozen/not frozen

        //             if (coord1.is_frozen == 1) {
        //                 VGA_rect(100, 10 * coord1.frozen_y + 100, 
        //                         100 + 10, 10 * coord1.frozen_y + 110, blue);
        //             } else {
        //                 VGA_rect(100, 10 * coord1.frozen_y + 100, 
        //                         100 + 10, 10 * coord1.frozen_y + 110, white);
        //             }
        //             if (coord2.is_frozen == 1) {
        //                 VGA_rect(100, 10 * coord2.frozen_y + 100, 
        //                         100 + 10, 10 * coord2.frozen_y + 110, blue);
        //             } else {
        //                 VGA_rect(100, 10 * coord2.frozen_y + 100, 
        //                         100 + 10, 10 * coord2.frozen_y + 110, white);
        //             }
        //             buffer_index++;      
        //         }
        //         // used for controlling the drawing speed
        //         usleep(500000); 

        //     // Reset buffer after drawing the snowflake
        //     buffer_index = 0; 
        //     pthread_cond_signal(&buffer_not_full); 

        
        }
        pthread_mutex_unlock(&buffer_mutex);
        *pio_done_send_addr = 1;

        *pio_done_send_addr = 0;

    }
    // while (1) { 

    //     // read values from the FPGA 
    //     y       = *pio_frozen_y_addr;
    //     y_froze = *pio_is_frozen_addr;

    //     *pio_done_send_addr = 1;

    //     if (y_froze == 1) { 
    //             VGA_rect(100, 10 * y + 100, 
    //                             100 + 10, 10 * y + 110, blue);

    //     }
    //     else { 
    //             VGA_rect(100, 10 * y + 100, 
    //                     100 + 10, 10 * y + 110, white);
    //     }

    //         *pio_done_send_addr = 1;

    //         *pio_done_send_addr = 0;

    // }

}


int p;


int main(void)
{

    // Declare volatile pointers to I/O registers (volatile     // means that IO load and store instructions will be used   // to access these pointer locations, 
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
    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 )    {
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
    vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, (   PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE ); 
    if( vga_char_virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap2() failed...\n" );
        close( fd );
        return(1);
    }
    
    // Get the address that maps to the FPGA LED control 
    vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

    // === get VGA pixel addr ====================
    // get virtual addr that maps to physical
    vga_pixel_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, (    PROT_READ | PROT_WRITE ), MAP_SHARED, fd,           FPGA_ONCHIP_BASE);  
    if( vga_pixel_virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap3() failed...\n" );
        close( fd );
        return(1);
    }
    
    // Get the address that maps to the FPGA pixel buffer
    vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

    // ===========================================

    // PIO POINTER STUFF
    // Maps to FPGA registers
    pio_alpha_addr   = (unsigned int *)(h2p_lw_virtual_base +  PIO_ALPHA_BASE );
    pio_beta_addr    = (unsigned int *)(h2p_lw_virtual_base +  PIO_BETA_BASE );
    pio_gamma_addr   = (unsigned int *)(h2p_lw_virtual_base +  PIO_GAMMA_BASE );
    pio_is_frozen_addr = (unsigned int *)(h2p_lw_virtual_base +  PIO_IS_FROZEN_BASE );
    pio_frozen_y_addr = (unsigned int *)(h2p_lw_virtual_base +  PIO_FROZEN_Y_BASE );
    pio_reset_to_addr = (unsigned int *)(h2p_lw_virtual_base +  PIO_RESET_TO_BASE );
    pio_done_send_addr = (unsigned int *)(h2p_lw_virtual_base +  PIO_DONE_SEND_BASE );

    /// VISUALIZE ON THE SCREEN /// 

    char text_x[40] = "init alpha = ";
    char text_y[40] = "init beta = ";
    char text_z[40] = "init gamma = ";
    /* create a message to be displayed on the VGA 
          and LCD displays */
    char text_top_row[40]    = "DE1-SoC ARM/FPGA\0";
    char text_bottom_row[40] = "Cornell ece5760\0";

    // VGA_box (0, 0, 639, 479, 0x0000);

    VGA_text (34, 1, text_top_row);
    VGA_text (34, 2, text_bottom_row);
    VGA_text (34, 3, text_x);
    VGA_text (34, 4, text_y);
    VGA_text (34, 5, text_z);

    // Initial values to send to fpga 
    *pio_alpha_addr = float2fix18(temp_alpha);
    *pio_beta_addr  = float2fix18(temp_beta);
    *pio_gamma_addr = float2fix18(temp_gamma);

    while (1) { 

        // read values from the FPGA 
        y       = *pio_frozen_y_addr;
        y_froze = *pio_is_frozen_addr;

        *pio_done_send_addr = 1;

        if (y_froze == 1) { 
                VGA_rect(100, 10 * y + 100, 
                                100 + 10, 10 * y + 110, blue);

        }
        else { 
                VGA_rect(100, 10 * y + 100, 
                        100 + 10, 10 * y + 110, white);
        }

            *pio_done_send_addr = 1;

            *pio_done_send_addr = 0;

    }

    // pthread_mutex_init(&buffer_mutex, NULL);

    // thread identifiers
    // pthread_t thread_frozen, thread_draw, thread_debug;

    // pthread_attr_t attr;
    // pthread_attr_init( &attr );
    // pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );
    
     // now the threads
    // pthread_create( &thread_reset, NULL,    reset_thread, NULL );
    // pthread_create( &thread_scan, NULL,     scan_thread,  NULL );
    // pthread_create( &thread_debug, NULL,     debug_thread, NULL );
    // pthread_create( &thread_frozen, NULL,   frozen_thread, NULL );
    // pthread_create( &thread_draw, NULL,     draw_thread, NULL );

    // pthread_join( thread_reset, NULL );
    // pthread_join( thread_scan, NULL );
    // pthread_join( thread_debug,  NULL );
    // pthread_join( thread_frozen, NULL );
    // pthread_join( thread_draw,   NULL );

    // pthread_mutex_destroy(&buffer_mutex);

	// printf(buffer_index);
    // for ( i = 0; i < buffer_index; ++i) {  // Only iterate up to the current buffer index
    //     printf("idx %d: is_frozen = %d, frozen_y = %d\n", i, buffer[i].is_frozen, buffer[i].frozen_y);
    // }

    return 0;

    // initialize_grid();

    // VGA_text (10, 1, text_top_row);
    // VGA_text (10, 2, text_bottom_row);
    
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
    volatile char * character_buffer = (char *) vga_char_ptr ;  // VGA character buffer
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
    volatile char * character_buffer = (char *) vga_char_ptr ;  // VGA character buffer
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
 * Draw an UNFILLED rectangle on the VGA monitor 
****************************************************************************************/
void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color)
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
    for (row = y1; row <= y2; row++){
        if (row == y1 || row == y2){
            for (col = x1; col <= x2; ++col)
            {
                //640x480
                pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
                // set pixel color
                *(char *)pixel_ptr = pixel_color;       
            }
        }
        else{
            //640x480
            pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + x1 ;
            // set pixel color
            *(char *)pixel_ptr = pixel_color;   
            //640x480
            pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + x2 ;
            // set pixel color
            *(char *)pixel_ptr = pixel_color;   
        }
    }
}