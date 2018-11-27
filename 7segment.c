/*
 * Raspberry Pi 4 digit 7 segment display effects without GPIO libraries
 * Copyright Daniel Jones <daniel@danieljon.es> 2018
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
   diagram
   =======
     A
     _
   F| |B
   G -
   E| |C
     -
     D

   GPIO memory mapping and associated snippets of code from Gert van Loo & Dom
   https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access
 */


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <ctype.h>
#include <time.h>
#include <string.h>

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

/*
 * pins:
 * real	represents
 * 17 	-> digit 0
 * 18 	-> digit 1
 * 27	-> digit 2
 * 22	-> digit 4
 * 23	-> segment A
 * 24	-> segment B
 * 25	-> segment C
 * 4	-> segment D
 * 2	-> segment E
 * 3	-> segment F
 * 8	-> segment G
 * 7	-> segment DOT
 */

enum segments
{
	/* refer to the diagram to know which value represents which segment */
	SEG_A 	= 1 << 0,
	SEG_B 	= 1 << 1,
	SEG_C	= 1 << 2,
	SEG_D	= 1 << 3,
	SEG_E	= 1 << 4,
	SEG_F	= 1 << 5,
	SEG_G	= 1 << 6,
	SEG_DOT	= 1 << 7,
};

enum segmentpins
{
	PIN_SELECT1 	= 17,
	PIN_SELECT2	= 18,
	PIN_SELECT3	= 27,
	PIN_SELECT4 	= 22,
	PIN_A		= 23,
	PIN_B		= 24,
	PIN_C		= 25,
	PIN_D		= 4,
	PIN_E		= 2,
	PIN_F		= 3,
	PIN_G		= 8,
	PIN_DOT		= 7,
};

enum characters
{
	CHAR_ZERO	= SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
	CHAR_ONE	= SEG_B | SEG_C,
	CHAR_TWO	= SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,
	CHAR_THREE	= SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,
	CHAR_FOUR	= SEG_B | SEG_C | SEG_F | SEG_G,
	CHAR_FIVE	= SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
	CHAR_SIX	= SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
	CHAR_SEVEN	= SEG_A | SEG_B | SEG_C,
	CHAR_EIGHT	= SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
	CHAR_NINE	= SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,
	CHAR_A		= SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,
	CHAR_B		= SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
	CHAR_C		= SEG_A | SEG_D | SEG_E | SEG_F,
	CHAR_D		= SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,
	CHAR_E		= SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,
	CHAR_F		= SEG_A | SEG_E | SEG_F | SEG_G,
	CHAR_G		= SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,
	CHAR_H		= SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,
	CHAR_I		= SEG_B | SEG_C,
	CHAR_J		= SEG_B | SEG_C | SEG_D | SEG_E,
	CHAR_L		= SEG_D | SEG_E | SEG_F,
	CHAR_N		= SEG_C | SEG_E | SEG_G,
	CHAR_O		= SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
	CHAR_P		= SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,
	CHAR_R		= SEG_E | SEG_G,
	CHAR_S		= SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
	CHAR_T		= SEG_D | SEG_E | SEG_F | SEG_G,
	CHAR_U		= SEG_C | SEG_D | SEG_E,
	CHAR_Y		= SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,
	CHAR_Z		= SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,
	CHAR_DOT	= SEG_DOT,
};

int digits[4] =
{
	PIN_SELECT1,
	PIN_SELECT2,
	PIN_SELECT3,
	PIN_SELECT4,
};

char hexcharacters[16] =
{CHAR_ZERO,
	CHAR_ONE,
	CHAR_TWO,
	CHAR_THREE,
	CHAR_FOUR,
	CHAR_FIVE,
	CHAR_SIX,
	CHAR_SEVEN,
	CHAR_EIGHT,
	CHAR_NINE,
	CHAR_A,
	CHAR_B,
	CHAR_C,
	CHAR_D,
	CHAR_E,
	CHAR_F
};

char ascii[127] =
{
	[46] = 0,
	CHAR_DOT,
	0,
	CHAR_ZERO,
	CHAR_ONE,
	CHAR_TWO,
	CHAR_THREE,
	CHAR_FOUR,
	CHAR_FIVE,
	CHAR_SIX,
	CHAR_SEVEN,
	CHAR_EIGHT,
	CHAR_NINE,
	[31] = 0,
	0, /* a space */
	[64] = 0,
	CHAR_A,
	CHAR_B,
	CHAR_C,
	CHAR_D,
	CHAR_E,
	CHAR_F,
	CHAR_G,
	CHAR_H,
	CHAR_I,
	CHAR_J,
	0,
	CHAR_L,
	0,
	CHAR_N,
	CHAR_O,
	CHAR_P,
	0,
	CHAR_R,
	CHAR_S,
	CHAR_T,
	CHAR_U,
	0,
	0,
	0,
	CHAR_Y,
	CHAR_Z,
};

void
setup_io(void)
{
	/* open /dev/gpiomem */
	if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC) ) < 0) {
		printf("can't open /dev/mem \n");
		exit(-1);
	}

	/* mmap GPIO */
	gpio_map = mmap(
			NULL,             //Any adddress in our space will do
			BLOCK_SIZE,       //Map length
			PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
			MAP_SHARED,       //Shared with other processes
			mem_fd,           //File to map
			GPIO_BASE         //Offset to GPIO peripheral
		       );

	close(mem_fd); //No need to keep mem_fd open after mmap

	if (gpio_map == MAP_FAILED) {
		printf("mmap error %d\n", (int)gpio_map);//errno also set!
		exit(-1);
	}

	// Always use volatile pointer!
	gpio = (volatile unsigned *)gpio_map;
}

void
displaycharacter(unsigned char segs)
{
	(segs & SEG_A) ? GPIO_SET = 1 << PIN_A : (GPIO_CLR = 1 << PIN_A);
	(segs & SEG_B) ? GPIO_SET = 1 << PIN_B : (GPIO_CLR = 1 << PIN_B);
	(segs & SEG_C) ? GPIO_SET = 1 << PIN_C : (GPIO_CLR = 1 << PIN_C);
	(segs & SEG_D) ? GPIO_SET = 1 << PIN_D : (GPIO_CLR = 1 << PIN_D);
	(segs & SEG_E) ? GPIO_SET = 1 << PIN_E : (GPIO_CLR = 1 << PIN_E);
	(segs & SEG_F) ? GPIO_SET = 1 << PIN_F : (GPIO_CLR = 1 << PIN_F);
	(segs & SEG_G) ? GPIO_SET = 1 << PIN_G : (GPIO_CLR = 1 << PIN_G);
	(segs & SEG_DOT) ? GPIO_SET = 1 << PIN_DOT : (GPIO_CLR = 1 << PIN_DOT);
}

void
clearpins(void)
{
	GPIO_SET = 1 << PIN_SELECT1;
	GPIO_SET = 1 << PIN_SELECT2;
	GPIO_SET = 1 << PIN_SELECT3;
	GPIO_SET = 1 << PIN_SELECT4;
	GPIO_CLR = 1 << PIN_A;
	GPIO_CLR = 1 << PIN_B;
	GPIO_CLR = 1 << PIN_C; 
	GPIO_CLR = 1 << PIN_D;
	GPIO_CLR = 1 << PIN_E;
	GPIO_CLR = 1 << PIN_F;
	GPIO_CLR = 1 << PIN_G;
	GPIO_CLR = 1 << PIN_DOT;
}

void
pinsetup(void)
{
	INP_GPIO(PIN_SELECT1);
	OUT_GPIO(PIN_SELECT1);
	INP_GPIO(PIN_SELECT2);
	OUT_GPIO(PIN_SELECT2);
	INP_GPIO(PIN_SELECT3);
	OUT_GPIO(PIN_SELECT3);
	INP_GPIO(PIN_SELECT4);
	OUT_GPIO(PIN_SELECT4);


	INP_GPIO(PIN_A);
	OUT_GPIO(PIN_A);
	INP_GPIO(PIN_B);
	OUT_GPIO(PIN_B);
	INP_GPIO(PIN_C);
	OUT_GPIO(PIN_C);
	INP_GPIO(PIN_D);
	OUT_GPIO(PIN_D);
	INP_GPIO(PIN_E);
	OUT_GPIO(PIN_E);
	INP_GPIO(PIN_F);
	OUT_GPIO(PIN_F);
	INP_GPIO(PIN_G);
	OUT_GPIO(PIN_G);
	INP_GPIO(PIN_DOT);
	OUT_GPIO(PIN_DOT);
	clearpins();
}

void
displayword(unsigned char str[4])
{
	for (int i = 0; i < 4; i++)
	{
		GPIO_SET = 1 << digits[0];
		GPIO_SET = 1 << digits[1];
		GPIO_SET = 1 << digits[2];
		GPIO_SET = 1 << digits[3];
		GPIO_CLR = 1 << digits[i];
		displaycharacter(ascii[toupper(str[i])]);
		usleep(1000);
	}
}

void
blinkword(unsigned char str[4], int repeat)
{
	clock_t start, end;
	start = clock();
	unsigned char original[5];
	unsigned char print[5];
	strcpy(original, str);
	strcpy(print, str);
	int state = 1;
	int repeats = 0;
	while (1)
	{
		if ((double)(end-start) >= 10000)
		{
			if (state)
				strcpy(print, "kkkkk");

			else
				strcpy(print, original);
			state = !state;
			if (repeat >= 0)
			{
				if (repeats >= (repeat*2)+1) /* *2 -> 5 flashes of the word */
					return;
				repeats++;
			}
			start = clock();
		}
		for (int i = 0; i < 4; i++)
		{
			GPIO_SET = 1 << digits[0];
			GPIO_SET = 1 << digits[1];
			GPIO_SET = 1 << digits[2];
			GPIO_SET = 1 << digits[3];
			GPIO_CLR = 1 << digits[i];
			displaycharacter(ascii[toupper(print[i])]);
			usleep(1000);
		}

		end = clock();
	}
}
int
converthour(int hour)
{
	switch (hour)
	{
		case 0:	 return 12;
		case 13: return 1;
		case 14: return 2;
		case 15: return 3;
		case 16: return 4;
		case 17: return 5;
		case 18: return 6;
		case 19: return 7;
		case 20: return 8;
		case 21: return 9;
		case 22: return 10;
		case 23: return 11;
		default: return hour;
	}
}

void
runclock(void)
{
	clock_t start, end;
	start = clock();
	unsigned char timenow[5] = "0000";
	while (1)
	{
		if ((double)(end-start) >= 50000)
		{
			time_t t = time(NULL);
			struct tm *tm = localtime(&t);
			if (converthour(tm->tm_hour) < 10)
				snprintf(timenow, 5, "k%d%02d", converthour(tm->tm_hour), tm->tm_min);
			else
				snprintf(timenow, 5, "%02d%02d", converthour(tm->tm_hour), tm->tm_min);
			start = clock();
		}
		for (int i = 0; i < 4; i++)
		{
			GPIO_SET = 1 << digits[0];
			GPIO_SET = 1 << digits[1];
			GPIO_SET = 1 << digits[2];
			GPIO_SET = 1 << digits[3];
			GPIO_CLR = 1 << digits[i];
			displaycharacter(ascii[toupper(timenow[i])]);
			usleep(1000);
		}
		end = clock();
	}

}

void
scrollword(unsigned char *string)
{
	size_t stringlen = strlen(string);
	char buffer[5] = "     ";
	clock_t start, end;
	start = clock();
	size_t wordsize = strlen(string);
	size_t buffsize = strlen(buffer);
	size_t circbuffsize = wordsize + buffsize;
	size_t position = 0; // start at character zero of word

	while (1)
	{
		if ((double)(end-start) >= 3000)
		{
		// https://stackoverflow.com/questions/48331360/scroll-a-word-through-an-array-in-c-from-right-to-left
			// shift the buffer
			for (size_t i = 0; i < (buffsize - 1); i++)
				buffer[i] = buffer[i + 1];
			// fill in the last character
			buffer[buffsize - 1] = position < wordsize ? string[position] : 'k';
			// increment the position in the imaginary circular source text
			position = (position + 1) % circbuffsize;
			start = clock();
		}
		for (int i = 0; i < 4; i++)
		{
			GPIO_SET = 1 << digits[0];
			GPIO_SET = 1 << digits[1];
			GPIO_SET = 1 << digits[2];
			GPIO_SET = 1 << digits[3];
			GPIO_CLR = 1 << digits[i];
			displaycharacter(ascii[toupper(buffer[i])]);
			usleep(1000);
		}
		end = clock();
	}

}

int
main(int argc, char *argv[])
{
	setup_io();
	pinsetup();
	//runclock();
	if (argv[1])
		scrollword(argv[1]);
	clearpins();
	return 0;
}

