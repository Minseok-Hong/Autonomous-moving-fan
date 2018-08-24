#ifndef __SEC_BUFFER_H__
#define __SEC_BUFFER_H__

#include <sys/types.h>
//! Buffer information
struct SecBuffer
{
    //! Buffer virtual address
    union {
        char *p;       //! single address.
        char *extP[3]; //! Y Cb Cr.
    } virt;

    //! Buffer physical address
    union {
        unsigned int p;       //! single address.
        unsigned int extP[3]; //! Y Cb Cr.
    } phys;

    //! Buffer reserved id
    union {
        unsigned int p;       //! \n
        unsigned int extP[3]; //! \n
    } reserved;

    //! Buffer size
    union {
        unsigned int s;
        unsigned int extS[3];
    } size;
};

#endif //__SEC_BUFFER_H__
