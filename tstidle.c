#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <bios.h>


extern void int_28( void );
#pragma aux int_28 =    \
    "int    28h"        \
    modify exact [];

extern void dpmi_idle( void );
#pragma aux dpmi_idle = \
    "mov    ax,1680h"   \
    "int    2Fh"        \
    modify exact [ax] nomemory;

struct {
    unsigned    dpmi_idle : 1;
    unsigned    kbd_poll  : 1;
    unsigned    kbd_wait  : 1;
    unsigned    idle_i28  : 1;
} Flags;

void usage( const char *arg0 )
{
    printf( "usage: %s [-k|-w] [-di] [-f fac]\n", arg0 );
    printf( "    -k      poll keyboard via INT 16h/01h\n"
            "    -w      wait for keyboard via INT 16h/00h\n"
            "    -d      call INT 2Fh/1680h DPMI idle\n"
            "    -f fac  busy work factor\n"
            "    -i      repeatedly call INT 28h\n" );
}


int do_work( int factor )
{
    unsigned long   counter = 1;
    int             i, j = 0;

    if( factor > 24 )
        factor = 24;

    counter <<= factor;

    while( counter-- ) {
        for( i = 1; i <= 100; ++i ) {
            j += j / i;
        }
    }

    return( j );
}

int main( int argc, char **argv )
{
    const char      *arg_0;
    unsigned short  key_state;
    int             factor;
    int             c;

    arg_0 = argv[0];
    while( (c = getopt( argc, argv, "f:dkiw" )) != -1 ) {
        switch( c ) {
        case 'd':
            Flags.dpmi_idle = 1;
            break;
        case 'k':
            Flags.kbd_poll = 1;
            break;
        case 'i':
            Flags.idle_i28 = 1;
            break;
        case 'w':
            Flags.kbd_wait = 1;
            break;
        case 'f':
            factor = atoi( optarg );
            break;
        case ':':
            printf( "%s: missing option argument\n", argv[0] );
            return( EXIT_FAILURE );
        default:
            usage( arg_0 );
            return( EXIT_FAILURE );
        }
    }

    if( Flags.kbd_wait ) {
        printf( "Waiting for keyboard input...\n" );
        key_state = _bios_keybrd( _KEYBRD_READ );
    } else if( Flags.kbd_poll ) {
        printf( "Polling keyboard. Esc to exit...\n" );
        key_state = _bios_keybrd( _KEYBRD_READY );
        while( (key_state >> 8) != 1 ) {

            do_work( factor );

            if( Flags.idle_i28 )
                int_28();
            if( Flags.dpmi_idle )
                dpmi_idle();

            key_state = _bios_keybrd( _KEYBRD_READY );
            /* Drain buffer if it's not empty. */
            if( key_state )
                _bios_keybrd( _KEYBRD_READ );
        }
    } else {
        printf( "Nothing to do.\n" );
    }

    return( EXIT_SUCCESS );
}
