/******************************************************************************
 *
 * FILE           : dbg.h
 * AUTHOR         : Emanuele
 * DATE           : 16/04/2007
 *
 * @FILE-ID       :
 * @VERSION       :
 * @RELEASE       :
 * @NETWORK-ENTITY:
 * @PROCESSOR     :
 * SUBSYSTEM      : drv
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License. (gpl)
 *
 * You should have received a copy of the GNU General Public License
 * (for example /usr/src/linux/COPYING); if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.  
 *
 *
 * REMARKS        : none
 *
 ******************************************************************************
 *
 * HISTORY         :
 *                   16/04/2007 - First release, by Author.
 *                   ... - ...
 *
 * [<MOD-ID>       :]
 *      Name       : ...................................................
 *      Date       : ...................................................
 *     [RFC        : ...................................................]
 *      Description: ...................................................
 *                   ...................................................
 *
 *****************************************************************************/

/* Debug header */
/* We are just an inclusion header file, check if we do not get in twice */
#ifndef drv_Debug_H
#define drv_Debug_H
/* Macroes for debugging in kernel space */
/* macroes are now drived by the value of debugOn , so it is possible by the
 * use of a simple ioct enable debugging output or other behavior while driver
 * is running. Note that if the driver is compiled with __DEBUG flag debugOn
 * default values will be 50  */
extern unsigned int _actOnHW;
extern unsigned int _debugOn;

#define debugPrint(who,fmt, args...)   debugPrintL(50, who,fmt, ## args)
#define debugPrintF(who,fmt, args...)  debugPrintFL(50, who,fmt, ## args)

#define debugPrintL(function_debug_level,who, fmt, args...)    do { if(function_debug_level <= _debugOn) printk( KERN_DEBUG "%s(L%i): - " fmt,who, _debugOn, ## args);  } while(0)
#define debugPrintFL(function_debug_level,who, fmt, args...)   do { if(function_debug_level <= _debugOn) printk( KERN_DEBUG "%s(L%i): - %s(ln %i) " fmt,who, _debugOn, __FUNCTION__, __LINE__, ## args);  } while(0)

#define printkE(who,fmt, args...)    printk( KERN_ERR "%s error: - %s(ln %i) " fmt,who, __FUNCTION__, __LINE__, ## args)
#define printkW(who,fmt, args...)    printk( KERN_ALERT "%s warning: - %s(ln %i) " fmt,who, __FUNCTION__, __LINE__, ## args)


#endif   /* drv_Debug_H */

