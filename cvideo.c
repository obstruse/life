/*-----------------------------------------------
 * Pico-Composite8      Composite Video, 8-bit output
 *
 * 2021-06-04           obstruse@earthlink.net
 *-----------------------------------------------
*/

#include <stdlib.h>
//#include <stdio.h>
#include "memory.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

void cvideo_configure_pio_dma(PIO pio, uint sm, uint dma_channel, size_t buffer_size_words);
void cvideo_dma_handler(void);
#include "cvideo.pio.h"     // The assembled PIO code

#define width 512           // Bitmap width in pixels		--- but there will be 2 pixels per byte
#define height 384          // Bitmap height in pixels

/*-------------------------------------------------------------------*/
/*------------------Video Standard-----------------------------------*/
/*-------------------------------------------------------------------*/

const int   VIDEO_frame_lines = 525;
const int   VIDEO_frame_lines_visible = 480;
const float VIDEO_aspect_ratio = 4.0/3.0;
//const float VIDEO_horizontal_freq = 15750.0;
const float VIDEO_horizontal_freq = 15734.0;
const float VIDEO_h_FP_usec = 1.5;	// front porch
const float VIDEO_h_SYNC_usec = 4.7;	// sync
const float VIDEO_h_BP_usec = 4.7;	// back porch
const float VIDEO_h_EP_usec = 2.3;	// equalizing pulse

/*-------------------------------------------------------------------*/
/*------------------Horizontal Derived-------------------------------*/
/*-------------------------------------------------------------------*/

const int   HORIZ_visible_dots = VIDEO_frame_lines_visible * VIDEO_aspect_ratio;	// full frame width
const float HORIZ_usec = 1000000.0 / VIDEO_horizontal_freq;
const float HORIZ_usec_dot = (HORIZ_usec - VIDEO_h_FP_usec - VIDEO_h_SYNC_usec - VIDEO_h_BP_usec) / HORIZ_visible_dots;
const int   HORIZ_dots = HORIZ_usec / HORIZ_usec_dot;

const int   HORIZ_bytes = HORIZ_dots / 2;	// two dots per byte
const int   HORIZ_FP_bytes = VIDEO_h_FP_usec / HORIZ_usec_dot / 2;
const int   HORIZ_SYNC_bytes = VIDEO_h_SYNC_usec / HORIZ_usec_dot / 2;
const int   HORIZ_BP_bytes = VIDEO_h_BP_usec / HORIZ_usec_dot / 2;
const int   HORIZ_EP_bytes = VIDEO_h_EP_usec / HORIZ_usec_dot / 2;	// equalizing pulse during vertical sync
const int   HORIZ_pixel_start = ((HORIZ_visible_dots - width) / 2) / 2 + HORIZ_SYNC_bytes + HORIZ_BP_bytes;

/*-------------------------------------------------------------------*/
/*------------------Vertical Derived---------------------------------*/
/*-------------------------------------------------------------------*/

const int   VERT_scanlines = VIDEO_frame_lines / 2;				// one field
const int   VERT_vblank    = (VIDEO_frame_lines - VIDEO_frame_lines_visible) / 2;	// vertical blanking, one field
const int   VERT_border    = (VERT_scanlines - VERT_vblank - height/2) / 2;
const int   VERT_bitmap   = height/2;

/*-------------------------------------------------------------------*/
/*------------------PIO----------------------------------------------*/
/*-------------------------------------------------------------------*/

const float PIO_clkdot = 1.0;        	// PIO instructions per dot
const float PIO_sysclk = 125000000.0;	// default Pico system clock
const float PIO_clkdiv = PIO_sysclk / VIDEO_horizontal_freq / PIO_clkdot / HORIZ_dots;

/*-------------------------------------------------------------------*/
/*------------------Gray Scale---------------------------------------*/
/*-------------------------------------------------------------------*/
// NTSC in IRE units+40: SYNC = 0; BLANK = 40; BLACK = 47.5; WHITE = 140
const int WHITE = 15;
const int BLACK = 15.0 / 140.0 * 47.5;
const int BLANK = 15.0 / 140.0 * 40.0;
const int GRAY  = (WHITE + BLACK)/ 3;
const int SYNC = 0;

#define border_colour BLACK

#define state_machine 0     // The PIO state machine to use
uint dma_channel;           // DMA channel for transferring hsync data to PIO

uint vline = 9999;          // Current video line being processed
uint bline = 0;             // Line in the bitmap to fetch
uint field = 1;		    // field, even/odd
uint lifeGen = 0;	    // alternating life generations

int bmIndex = 0;
int bmCount = 0;

// bitmap buffer
unsigned char * life[2];

unsigned char * vsync_ll;                             // buffer for a vsync line with a long/long pulse
unsigned char * vsync_ss;                             // Buffer for an equalizing line with a short/short pulse
unsigned char * vsync_bb;                             // Buffer for a vsync blanking
unsigned char * vsync_ssb;                            // Buffer and a half for equalizing/blank line
unsigned char * border;                               // Buffer for a vsync line for the top and bottom borders
unsigned char * pixel_buffer[2];                      // Double-buffer for the pixel data scanlines

volatile bool changeBitmap  = false;

/*-------------------------------------------------------------------*/
void setLifeCell(uint g, uint x, uint y, uint v) {
	uint LorR = x & 0x01;
	uint Lx = x/2;
	uint i = y*width/2 + Lx;
	uint Lv = v > 0 ? WHITE : BLACK;
	if ( LorR ) { 
	    *(life[g]+i) = (*(life[g]+i) & 0xf0) | Lv; 
	} else { 
	    *(life[g]+i) = (*(life[g]+i) & 0x0f) | Lv<<4; 
	}
}

/*-------------------------------------------------------------------*/
int getLifeCell(uint g, int x, int y) {
	// handle wrap-around
	//if ( y >= height ) y -= height;
	//if ( y < 0 ) y += height;
	//if ( x >= width ) x -= width;
	//if ( x < 0 ) x += width;

	// don't handle wrap-around
	if ( y >= height || y < 0 ) return 0;
	if ( x >= width  || x < 0 ) return 0;

        uint LorR = x & 0x01;
        uint Lx = x/2;
        uint i = y*width/2 + Lx;
	uint Lv;
        if ( LorR ) { 
	    Lv = (*(life[g]+i) & 0x0f); 
	} else { 
	    Lv = (*(life[g]+i) & 0xf0) >>4; 
	}
	return( Lv == WHITE );
}

/*-------------------------------------------------------------------*/
void second_core() {

uint lifeNext = 1;
uint lifeCounter = 0;
uint lifeBit = 0;
uint lifeByte = 0;
uint64_t emptyGlob;

    for ( int i=0; i<sizeof(emptyGlob); i++ ) {
	emptyGlob = (emptyGlob<<4)|BLACK;
	emptyGlob = (emptyGlob<<4)|BLACK;
    }

// seed
//#include "gosperglidergun.h"
//#include "bunnies.h"
//#include "blom.h"
//#include "homer.h"
#include "multuminparvo.h"

    int lastCol = 0;
    int thisCol = 0;
    int nextCol = 0;

    while (true) {
	    // work on the next generation
	    lifeNext = (lifeGen + 1) & 0x01;

	    // for every cell, count neighbors in current generation
	    // ...for each row
	    for ( int row = 1; row < height-1; row++ ) {

		// get the globs ready
		uint64_t * glob1 = (uint64_t *)(life[lifeGen]+((row-1)*width/2));
		uint64_t * glob2 = (uint64_t *)(life[lifeGen]+((row)*width/2));
		uint64_t * glob3 = (uint64_t *)(life[lifeGen]+((row+1)*width/2));
		uint64_t * glob4 = (uint64_t *)(life[lifeNext]+((row)*width/2));

		// sum the cols...
		lastCol = getLifeCell(lifeGen,-1,row-1) + getLifeCell(lifeGen,-1,row) + getLifeCell(lifeGen,-1,row+1);

		// do each glob on row
		int col = 0;
		for ( int g = 0; g < width/16; g++ ) {
		    int c = 0;

		    if ( lastCol == 0 && *(glob1+g) == emptyGlob && *(glob2+g) == emptyGlob && *(glob3+g) == emptyGlob ) {
			// there's nothing to process, skip to end of glob
			*(glob4+g) = emptyGlob;
			c = 15;
			col += c;
		    }

    		    thisCol = getLifeCell(lifeGen,col,row-1) + getLifeCell(lifeGen,col,row) + getLifeCell(lifeGen,col,row+1);
	    	    for ( ; c < 16; c++ ) {
			
		        nextCol = getLifeCell(lifeGen,col+1,row-1) + getLifeCell(lifeGen,col+1,row) + getLifeCell(lifeGen,col+1,row+1);
		        int currentCell = getLifeCell(lifeGen, col, row);
		        int liveNeighbors = lastCol + thisCol + nextCol - currentCell;

		        setLifeCell(lifeNext, col,  row, currentCell);
		        if ( liveNeighbors < 2 ) setLifeCell(lifeNext, col, row, 0);
		        if ( liveNeighbors > 3 ) setLifeCell(lifeNext, col, row, 0);
		        if ( liveNeighbors == 3 ) setLifeCell(lifeNext, col, row, 1);

                        lastCol = thisCol;
                        thisCol = nextCol;
			col++;
		    }

		}
	    }


	    lifeGen = lifeNext;

	    //sleep_ms(1000);		// sleep_ms() causes a visible glitch
	    //busy_wait_us(1000000);
    }
}

/*-------------------------------------------------------------------*/
int main() {
//	stdio_init_all();
//	sleep_ms(2000);
//	printf("Start of program\n");

    // for debug
    gpio_init(8);
    gpio_set_dir(8,GPIO_OUT);
    gpio_init(9);
    gpio_set_dir(9,GPIO_OUT);
    gpio_init(10);
    gpio_set_dir(10,GPIO_OUT);

    // two pixels(dots) per byte
    uint BLANK2 = BLANK | BLANK << 4;
    uint BLACK2 = BLACK | BLACK << 4;
    uint GRAY2  = GRAY  | GRAY  << 4;
    uint WHITE2 = WHITE | WHITE << 4;

    life[0] = (unsigned char *)malloc(width*height/2);
    memset(life[0], BLACK2, height*width/2);
    life[1] = (unsigned char *)malloc(width*height/2);
    memset(life[1], BLACK2, height*width/2);

    multicore_launch_core1(second_core);

    // two pixels(dots) per byte
    vsync_ll = (unsigned char *)malloc(HORIZ_bytes);
    memset(vsync_ll, SYNC, HORIZ_bytes);				// vertical sync/serrations
    memset(vsync_ll + (HORIZ_bytes>>1) - HORIZ_EP_bytes, BLANK2, HORIZ_EP_bytes);
    memset(vsync_ll + HORIZ_bytes      - HORIZ_EP_bytes, BLANK2, HORIZ_EP_bytes);

    vsync_ss = (unsigned char *)malloc(HORIZ_bytes);
    memset(vsync_ss, BLANK2, HORIZ_bytes);				// vertical equalizing
    memset(vsync_ss, SYNC, HORIZ_EP_bytes);
    memset(vsync_ss + (HORIZ_bytes>>1), SYNC, HORIZ_EP_bytes);

    vsync_bb = (unsigned char *)malloc(HORIZ_bytes);
    memset(vsync_bb, BLANK2, HORIZ_bytes);				// vertical blanking
    memset(vsync_bb, SYNC, HORIZ_SYNC_bytes);

    vsync_ssb = (unsigned char *)malloc(HORIZ_bytes+(HORIZ_bytes>>1));
    memset(vsync_ssb, BLANK2, HORIZ_bytes + (HORIZ_bytes>>1));		// vertical equalizing/blanking
    memset(vsync_ssb, SYNC, HORIZ_EP_bytes);
    memset(vsync_ssb + (HORIZ_bytes>>1), SYNC, HORIZ_EP_bytes);

    // This bit pre-builds the border scanline and pixel buffers
    border = (unsigned char *)malloc(HORIZ_bytes);
    memset(border, GRAY2, HORIZ_bytes);			// Fill the border with the border colour
    memset(border, SYNC, HORIZ_SYNC_bytes);		        // Add the hsync pulse
    memset(border + HORIZ_SYNC_bytes,            BLANK2, HORIZ_BP_bytes);
    memset(border + HORIZ_bytes - HORIZ_FP_bytes, BLANK2, HORIZ_FP_bytes);		// front porch

    pixel_buffer[0] = (unsigned char *)malloc(HORIZ_bytes);
    memcpy(pixel_buffer[0], border, HORIZ_bytes);			// pixel buffer
    pixel_buffer[1] = (unsigned char *)malloc(HORIZ_bytes);
    memcpy(pixel_buffer[1], border, HORIZ_bytes);			// pixel buffer

    // Initialise the PIO
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &cvideo_program);	// Load up the PIO program
    pio_sm_set_enabled(pio, state_machine, false);              // Disable the PIO state machine
    pio_sm_clear_fifos(pio, state_machine);	                // Clear the PIO FIFO buffers
    cvideo_initialise_pio(pio, state_machine, offset, 4, 4, PIO_clkdiv); // Initialise the PIO (function in cvideo.pio)

    dma_channel = dma_claim_unused_channel(true);		// Claim a DMA channel for the hsync transfer
    cvideo_configure_pio_dma(pio, state_machine, dma_channel, HORIZ_bytes); // Hook up the DMA channel to the state machine


    // And kick everything off
    cvideo_dma_handler();       // Call the DMA handler as a one-off to initialise it
    pio_sm_set_enabled(pio, state_machine, true);               // Enable the PIO state machine

    while (true) {              // And then just loop doing nothing
	tight_loop_contents();
    }
}

/*-------------------------------------------------------------------*/
// The DMA interrupt handler
// This is triggered by DMA_IRQ_0

void cvideo_dma_handler(void) {

    if ( ++vline <= VERT_scanlines ) {
    } else {
        vline = 0;
	bline = 0;
	field = ++field & 0x01;
    }

    while (true) {
        if ( vline <= VERT_vblank ) {

            switch(vline) {

            case 0:
		if ( field ) {
			// odd field - blank, full line
			dma_channel_set_read_addr(dma_channel, vsync_bb, true);

                } else {
			// even field - blank, half line
			dma_channel_set_trans_count(dma_channel, HORIZ_bytes/2, false);
			dma_channel_set_read_addr(dma_channel, vsync_bb, true);
                }

		break;

            case 1:
		dma_channel_set_trans_count(dma_channel, HORIZ_bytes, false);   // reset transfer size
            case 2 ... 3:
                // send 3 vsync_ss - 'equalizing pulses'
                dma_channel_set_read_addr(dma_channel, vsync_ss, true);

                break;

            case 4 ... 6:
                // send 3 vsync_ll - 'vertical sync/serrations'
                dma_channel_set_read_addr(dma_channel, vsync_ll, true);

                break;

            case 7 ... 8:
                // send 3 vsync_ss - 'equalizing pulses'
                dma_channel_set_read_addr(dma_channel, vsync_ss, true);

                break;

	    case 9:
		if ( field ) {
			// odd field - equalizing pulse, full line
			dma_channel_set_read_addr(dma_channel, vsync_ss, true);

           	} else {
			//even field - equalizing pulse, line and a half
			dma_channel_set_trans_count(dma_channel, HORIZ_bytes + HORIZ_bytes/2, false);
			dma_channel_set_read_addr(dma_channel, vsync_ssb, true);
     		}

		break;

            case 10:
		// everything back to normal
		dma_channel_set_trans_count(dma_channel, HORIZ_bytes, false);   // reset transfer size
	    default:
                // send BLANK till end of vertical blanking
                dma_channel_set_read_addr(dma_channel, vsync_bb, true);

                break;

            }

            break;
        }

        if ( vline <= VERT_vblank + VERT_border ) {

            if (changeBitmap) {
                dma_channel_set_read_addr(dma_channel, vsync_bb, true);
            } else {
                dma_channel_set_read_addr(dma_channel, border, true);
		if ( vline == VERT_vblank + VERT_border ) {
		    memcpy(pixel_buffer[bline & 1] + HORIZ_pixel_start, life[lifeGen]+(bline*2+field)*width/2, width/2);
		}

            }

            break;
        }

        if ( vline <= VERT_vblank + VERT_border + VERT_bitmap  ) {

	    if (changeBitmap) {
                dma_channel_set_read_addr(dma_channel, vsync_bb, true);
	    } else {
                dma_channel_set_read_addr(dma_channel, pixel_buffer[bline++ & 1], true);    // Set the DMA to read from one of the pixel_buffers
                memcpy(pixel_buffer[bline & 1] + HORIZ_pixel_start, life[lifeGen]+(bline*2+field)*width/2, width/2);       // And memcpy the next scanline
	    }
            break;
        }

        // otherwise, just output border until end of scanlines
	if (changeBitmap) {
            dma_channel_set_read_addr(dma_channel, vsync_bb, true);
	} else {
            dma_channel_set_read_addr(dma_channel, border, true);
	}

        break;
    }

    // Finally, clear the interrupt request ready for the next horizontal sync interrupt
    dma_hw->ints0 = 1u << dma_channel;
}

/*-------------------------------------------------------------------*/
// Configure the PIO DMA
// Parameters:
// - pio: The PIO to attach this to
// - sm: The state machine number
// - dma_channel: The DMA channel
// - buffer_size_words: Number of bytes to transfer
//
void cvideo_configure_pio_dma(PIO pio, uint sm, uint dma_channel, size_t buffer_size_words) {
    pio_sm_clear_fifos(pio, sm);
    dma_channel_config c = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(dma_channel, &c,
                          &pio->txf[sm],              // Destination pointer
                          NULL,                       // Source pointer
                          buffer_size_words,          // Number of transfers
                          false                        // Start flag (true = start immediately)
                         );

    dma_channel_set_irq0_enabled(dma_channel, true);

    irq_set_exclusive_handler(DMA_IRQ_0, cvideo_dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}
