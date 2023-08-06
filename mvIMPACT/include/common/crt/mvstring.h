//-----------------------------------------------------------------------------
// (C) Copyright 2005 - 2023 by MATRIX VISION GmbH
//
// This software is provided by MATRIX VISION GmbH "as is"
// and any express or implied warranties, including, but not limited to, the
// implied warranties of merchantability and fitness for a particular purpose
// are disclaimed.
//
// In no event shall MATRIX VISION GmbH be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused and
// on any theory of liability, whether in contract, strict liability, or tort
// (including negligence or otherwise) arising in any way out of the use of
// this software, even if advised of the possibility of such damage.

//-----------------------------------------------------------------------------
#ifndef mvstringH
#define mvstringH mvstringH
//-----------------------------------------------------------------------------
#include <errno.h>
#include <stdlib.h>
#include <string.h>

//-----------------------------------------------------------------------------
/// \brief \a Safe version of the pre-C11 strncpy, adhering to the C11 standard behavior of strncpy_s(pDst, bufsize, pSrc, bufsize)
/**
* Copies at most \c bufsize characters of the character array pointed to by \c pSrc (including the terminating null character, but not
* any of the characters that follow the null character) to character array pointed to by \c pDst.
* If \c bufsize is reached before the entire array \c pSrc was copied, this function writes a null at pDst[bufsize] and then stops.
* The behavior is undefined if the character arrays overlap, if either \c pDst or \c pSrc is not a pointer to a character array
* (including if \c pDst or \c pSrc is a null pointer), if the size of the array pointed to by \c pDst is less than bufsize, or if
* the size of the array pointed to by \c pSrc is less than \c bufsize and it does not contain a null character.
 * \ingroup CommonInterface
*/
inline int mv_strncpy_s( char* pDst, const char* pSrc, size_t bufSize )
//-----------------------------------------------------------------------------
{
#if (defined(_MSC_VER) && (_MSC_VER >= 1400)) // is at least VC 2005 compiler?
    // '_TRUNCATE' is a Microsoft extension!
    return strncpy_s( pDst, bufSize, pSrc, _TRUNCATE );
#elif defined (__STDC_LIB_EXT1__) // does implementation support CRT extensions?
    return strncpy_s( pDst, bufSize, pSrc, bufSize );
#else
    return pDst != strncpy( pDst, pSrc, bufSize );
#endif // #if (defined(_MSC_VER) && (_MSC_VER >= 1400)) // is at least VC 2005 compiler?
}

//-----------------------------------------------------------------------------
/// \brief Version that mimics the C11 \c strncat_s function.
/**
 * See \c strncat_s of your runtime implementation for documentation!
 * \ingroup CommonInterface
 */
inline int mv_strncat_s( char* pDst, const char* pSrc, size_t bufSize )
//-----------------------------------------------------------------------------
{
#if (defined(_MSC_VER) && (_MSC_VER >= 1400)) // is at least VC 2005 compiler?
    // '_TRUNCATE' is a Microsoft extension!
    return strncat_s( pDst, bufSize, pSrc, _TRUNCATE );
#elif defined (__STDC_LIB_EXT1__) // does implementation support CRT extensions?
    return strncat_s( pDst, bufSize, pSrc, bufSize - strnlen_s( pDst, bufSize ) );
#else
    return pDst != strncat( pDst, pSrc, bufSize );
#endif // #if (defined(_MSC_VER) && (_MSC_VER >= 1400)) // is at least VC 2005 compiler?
}

//-----------------------------------------------------------------------------
/// \brief Version that mimics the C11 \c strerror_s function.
/**
 * See \c strerror_s of your runtime implementation for documentation!
 * \ingroup CommonInterface
 */
inline int mv_strerror_s( char* pBuf, size_t bufSize, int errnum )
//-----------------------------------------------------------------------------
{
#if (defined(_MSC_VER) && (_MSC_VER >= 1400)) || defined (__STDC_LIB_EXT1__) // is at least VC 2005 compiler OR implementation supports CRT extensions?
    return strerror_s( pBuf, bufSize, errnum );
#else
    // Should check the following constraints here:
    // - pBuf is a null pointer
    // - bufSize is zero or greater than RSIZE_MAX
    return pBuf != strncpy( pBuf, strerror( errnum ), bufSize );
#endif // #if (defined(_MSC_VER) && (_MSC_VER >= 1400)) || defined (__STDC_LIB_EXT1__) // is at least VC 2005 compiler OR implementation supports CRT extensions?
}

//-----------------------------------------------------------------------------
/// \brief Version that mimics the C11 \c strerrorlen_s function.
/**
 * See \c strerrorlen_s of your runtime implementation for documentation!
 * \ingroup CommonInterface
 */
inline size_t mv_strerrorlen_s( int errnum )
//-----------------------------------------------------------------------------
{
#ifdef __STDC_LIB_EXT1__ // does implementation supports CRT extensions?
    return strerrorlen_s( errnum );
#else
#   if defined(__clang__)
#       pragma clang diagnostic push
#       pragma clang diagnostic ignored "-Wdeprecated-declarations"
#   elif defined(_MSC_VER) && (_MSC_VER >= 1800) // is at least VC 2013 compiler?
#       pragma warning( push )
#       pragma warning( disable : 4996 ) // 'warning C4996: 'GetVersionExA': was declared deprecated'
#   endif
    return strlen( strerror( errnum ) );
#   if defined(__clang__)
#       pragma clang diagnostic pop
#   elif defined(_MSC_VER) && (_MSC_VER >= 1800) // is at least VC 2013 compiler?
#       pragma warning( pop ) // restore old warning level
#   endif
#endif // #ifdef __STDC_LIB_EXT1__ // does implementation supports CRT extensions?
}

#endif // mvstringH
