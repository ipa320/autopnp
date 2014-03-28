/*
 * Copyright (c) 2011-2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: time_arch.c 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 *         Time abstraction (architecture specific part: Windows
 *         implementation).
 */

/*
 * Portions of this file are adapted from the discussion at
 * http://social.msdn.microsoft.com/Forums/en-US/vcgeneral/thread/430449b3-f6dd-4e18-84de-eebd26a8d668/
 * Licensing conditions of that portion according to http://msdn.microsoft.com/en-us/cc300389.aspx follow.
 *
 * ACCEPTANCE OF TERMS OF USE
 * Microsoft provides you with access to a variety of resources on this web site, including documents, photographs, videos, and other graphical, textual or audio-visual content ("Content"), software and computer code, including developer tools and sample code ("Software"), and communication forums and other services ("Services").  The Content, Software, Services and all other aspects of this web site are subject to these Terms of Use.  Microsoft reserves the right to update these Terms of Use at any time without notice to you.  The applicable and most current version of the Terms of Use can be reviewed by clicking on the "Terms of Use" hypertext link located at the bottom of our web pages.
 * By accessing or using this web site in any way, you agree to and are bound by the terms of this Terms of Use.  If you do not agree to all of the terms and conditions contained in the Terms of Use, do not access or use this web site.  
 *
 * PRIVACY AND PROTECTION OF PERSONAL INFORMATION
 * See the Privacy Statement for disclosures relating to the collection and use of your information.  
 *
 * APPLICATION PROGRAMMING INTERFACES
 * Microsoft publishes information on a number of application programming interfaces ("APIs") on this web site.  Microsoft will not assert any of its patent rights on account of your products calling these APIs in order to receive services from the Microsoft product that exposes the APIs. 
 *
 * SOFTWARE
 * All Software that is made available to download from the web site is the copyrighted work of Microsoft or its suppliers. Your use of Software is governed by the terms of the license agreement, if any, that accompanies or is included with the Software.  
 * If this web site provides any Software (such as javascript) to your computer's browser and does not include a license agreement, then Microsoft grants you the right to use that Software solely to interact through your browser with this web site.
 * If any Software contains a copyright notice or similar indication of ownership that indicates it is owned by someone other than Microsoft, and it includes its own license agreement, then that Software is licensed to you by that other party and not Microsoft, and Microsoft grants you no intellectual property rights (express or implied) with respect to that Software. 
 * If Microsoft makes any Software marked as "sample" or "example" available on this web site without a license agreement, then it is licensed to you under the terms of the Microsoft Limited Public License.  
 * If Microsoft makes any other Software available on this web site without a license agreement, you may use it solely to design, develop and test your programs to run on Microsoft products and services. 
 *
 * CONTENT
 * All Content is the copyrighted work of Microsoft or its suppliers. Use of the Content is governed by the terms of the license agreement, if any, that accompanies or is included with the Content.
 * If any Content is made available to you on this web site without a license agreement, then you may make a reasonable number of copies of the Content for your internal use in designing, developing, and testing your software, products and services. You must preserve the below copyright notice in all copies of the Content and ensure that both the copyright notice and this permission notice appear in those copies. 
 * Accredited educational institutions, such as K-12 schools, universities, private or public colleges, and state community colleges, may download and reproduce Content for distribution in the classroom for educational purposes. Publication or distribution outside the classroom requires express written permission.  
 * Except as provided above in this section, no portion of the web site may be copied, imitated, published, transmitted, broadcast or distributed, in whole or in part.  
 *
 * CONTENT ACCESSIBLE ONLY TO INVITED PARTICIPANTS
 *
 * Certain portions of this web site are accessible only to users who are invited to participate, for example as part of a program for using pre-release Software and providing feedback to Microsoft. All information available in those portions of this web site or concerning Content or Software available in those portions of this web site are confidential information of Microsoft. For a period of five years from the time you accessed this confidential information, you may not disclose this confidential information to any third party. This restriction will not apply to any information that is or becomes publicly available without a breach of this restriction, was lawfully known to the receiver of the information without an obligation to keep it confidential, is received from another source who can disclose it lawfully and without an obligation to keep it confidential, or is independently developed. You may disclose this confidential information if required to comply with a court order or other government demand that has the force of law.  Before doing so, you must seek the highest level of protection available and, when possible, give Microsoft enough prior notice to provide a reasonable chance to seek a protective order.
 *
 * NOTICES REGARDING SOFTWARE, CONTENT, APIs, SERVICES AND INFORMATION AVAILABLE ON THIS WEB SITE
 * THE SOFTWARE, CONTENT, APIS AND SERVICES, AND INFORMATION AVAILABLE FROM THIS WEB SITE OR THE SERVICES, ARE WARRANTED, IF AT ALL, ONLY ACCORDING TO THE TERMS OF A SEPARATE AGREEMENT THAT COVERS THE APPLICABLE SOFTWARE, CONTENT, APIS, SERVICES AND INFORMATION.  EXCEPT AS WARRANTED IN THAT SEPARATE AGREEMENT (IF ANY), MICROSOFT CORPORATION AND ITS RESPECTIVE SUPPLIERS HEREBY DISCLAIM ALL WARRANTIES AND CONDITIONS WITH REGARD TO THE SOFTWARE, CONTENT, APIS, SERVICES AND INFORMATION, INCLUDING ALL WARRANTIES AND CONDITIONS OF MERCHANTABILITY, WHETHER EXPRESS, IMPLIED OR STATUTORY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL MICROSOFT OR ITS SUPPLIERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF SOFTWARE, CONTENT, APIS, PROVISION OF OR FAILURE TO PROVIDE SERVICES, OR INFORMATION AVAILABLE FROM THE SERVICES OR THIS WEB SITE.
 * Microsoft may have patents, patent applications, trademarks, copyrights, or other intellectual property rights covering subject matter in the Content, Software or Services. Except as expressly provided in any written license agreement from Microsoft, the furnishing of Content, Software or Services does not give you any license to these patents, trademarks, copyrights, or other intellectual property.
 *
 * RESERVATION OF RIGHTS
 * Microsoft reserves all rights not expressly granted under these terms of use, and no other rights are granted under these terms of use by implication or estoppel or otherwise.
 *
 * MEMBER ACCOUNT, PASSWORD, AND SECURITY
 * If any of the Services requires you to open an account, you must complete the registration process by providing us with current, complete and accurate information as prompted by the applicable registration form. You are entirely responsible for maintaining the confidentiality of your password and account. Furthermore, you are entirely responsible for any and all activities that occur under your account. You agree to notify Microsoft immediately of any unauthorized use of your account or any other breach of security. Microsoft will not be liable for any loss that you may incur as a result of someone else using your password or account, either with or without your knowledge. However, you could be held liable for losses incurred by Microsoft or another party due to someone else using your account or password. You may not use anyone else's account without the permission of the account holder.
 *
 * PROHIBITED USE OF SERVICES
 * For any communities on this web site, you must follow the Microsoft Communities Rules of Conduct.
 * As a condition of your use of Services, you will not use them for any purpose that is unlawful or prohibited by these terms, conditions, and notices. You may not use the Services in any manner that could damage, disable, overburden, or impair any Microsoft server, or the network(s) connected to any Microsoft server, or interfere with any other party's use and enjoyment of any Services. You may not attempt to gain unauthorized access to any Services, other accounts, computer systems or networks connected to any Microsoft server or to any of the Services, through hacking, password mining or any other means. You may not obtain or attempt to obtain any materials or information through any means not intentionally made available through the Services.
 * The Services may contain e-mail services, bulletin board services, chat areas, forums, communities, personal web pages, calendars, photo albums, file cabinets and/or other message or communication facilities designed to enable you to communicate with others ("Communication Services"). You agree to use the Communication Services only to post, send and receive messages and material that are proper and, when applicable, related to the particular Communication Service. By way of example, and not as a limitation, you agree that when using the Communication Services, you will not:
 *    Use the Communication Services in connection with surveys, contests, pyramid schemes, chain letters, junk email, spamming or any duplicative or unsolicited messages (commercial or otherwise).
 *    Defame, abuse, harass, stalk, threaten or otherwise violate the legal rights (such as rights of privacy and publicity) of others.
 *    Publish, post, upload, distribute or disseminate any inappropriate, profane, defamatory, obscene, indecent or unlawful topic, name, material or information.
 *    Upload, or otherwise make available, files that contain images, photographs, software or other material protected by intellectual property laws, including, by way of example, and not as limitation, copyright or trademark laws (or by rights of privacy or publicity) unless you own or control the rights thereto or have received all necessary consent to do the same.
 *    Use any material or information, including images or photographs, which are made available through the Services in any manner that infringes any copyright, trademark, patent, trade secret, or other proprietary right of any party.
 *    Upload files that contain viruses, Trojan horses, worms, time bombs, cancelbots, corrupted files, or any other similar software or programs that may damage the operation of another's computer or property of another.
 *    Advertise or offer to sell or buy any goods or services for any business purpose, unless such Communication Services specifically allows such messages.
 *    Download any file posted by another user of a Communication Service that you know, or reasonably should know, cannot be legally reproduced, displayed, performed, and/or distributed in such manner.
 *    Falsify or delete any copyright management information, such as author attributions, legal or other proper notices or proprietary designations or labels of the origin or source of software or other material contained in a file that is uploaded.
 *    Restrict or inhibit any other user from using and enjoying the Communication Services.
 *    Violate any code of conduct or other guidelines which may be applicable for any particular Communication Service.
 *    Harvest or otherwise collect information about others, including e-mail addresses.
 *    Violate any applicable laws or regulations.
 *    Create a false identity for the purpose of misleading others.
 *    Use, download or otherwise copy, or provide (whether or not for a fee) to a person or entity any directory of users of the Services or other user or usage information or any portion thereof.
 * Microsoft has no obligation to monitor the Communication Services. However, Microsoft reserves the right to review materials posted to the Communication Services and to remove any materials in its sole discretion.
 * Microsoft reserves the right at all times to disclose any information as Microsoft deems necessary to satisfy any applicable law, regulation, legal process or governmental request, or to edit, refuse to post or to remove any information or materials, in whole or in part, in Microsoft's sole discretion.
 * Always use caution when giving out any personally identifiable information about yourself or your children and business sensitive information in any Communication Services.  Microsoft does not control or endorse the content, messages or information found in any Communication Services, and Microsoft specifically disclaims any liability with regard to the Communication Services. Managers and hosts are not authorized Microsoft spokespersons, and their views do not necessarily reflect those of Microsoft.
 * Materials uploaded to the Communication Services may be subject to posted limitations on usage, reproduction or dissemination. You are responsible for adhering to such limitations if you download the materials.
 *
 * SUBMISSIONS PROVIDED TO THIS WEB SITE
 * Microsoft does not claim ownership of code, content, comments, feedback, suggestions, information or materials that you provide via this web site or any Services ("Submission"). However, by providing a Submission, you are irrevocably granting Microsoft and its affiliated companies the right to make, use, modify, distribute and otherwise commercialize the Submission in any way and for any purpose (including by granting the general public the right to use your Submissions in accordance with this web site's Terms of Use, which may change over time), and the right to publish your name, city of residence, and e-mail address in connection with your Submission.  These rights are granted under all applicable intellectual property rights you own or control.
 * No compensation will be paid with respect to the use of your Submissions. Microsoft is under no obligation to post or use any Submission, and Microsoft may remove any Submission at any time.
 * By providing a Submission you warrant that you own or otherwise control all of the rights to your Submission and that your Submission is not subject to any rights of a third party (including any personality or publicity rights of any person).
 *
 * TERMINATION
 * Microsoft reserves the right to terminate your access to any or all of the Services at any time, without notice, for any reason whatsoever.
 *
 * NOTICES AND PROCEDURE FOR MAKING CLAIMS OF COPYRIGHT INFRINGEMENT
 * Pursuant to Title 17, United States Code, Section 512(c)(2), notifications of claimed copyright infringement should be sent to Service Provider's Designated Agent. ALL INQUIRIES NOT RELEVANT TO THE FOLLOWING PROCEDURE WILL NOT RECEIVE A RESPONSE.
 * See Notice and Procedure for Making Claims of Copyright Infringement.
 *
 * LINKS TO THIRD PARTY SITES
 * SOME LINKS ON THIS SITE WILL LET YOU LEAVE THE MICROSOFT WEB SITE. MICROSOFT IS PROVIDING THESE LINKS TO YOU ONLY AS A CONVENIENCE, AND THE INCLUSION OF ANY LINK DOES NOT IMPLY ENDORSEMENT BY MICROSOFT OF THE SITE.
 * YOU ACKNOWLEDGE AND AGREE THAT: (i) MICROSOFT DOES NOT CONTROL, REVIEW, REVISE, ENDORSE, OR ACCEPT RESPONSIBILITY FOR ANY MATERIALS, PROJECTS OR SERVICES OFFERED BY THIRD PARTIES, INCLUDING THIRD-PARTY VENDORS AND THIRD PARTIES ACCESSIBLE THROUGH LINKED SITES; (ii) MICROSOFT MAKES NO REPRESENTATIONS OR WARRANTIES WHATSOEVER ABOUT ANY SUCH THIRD PARTIES, THEIR MATERIALS OR SERVICES; (iii) ANY DEALINGS YOU MAY HAVE WITH SUCH THIRD PARTIES ARE AT YOUR OWN RISK; AND (iv) MICROSOFT SHALL NOT BE LIABLE OR RESPONSIBLE FOR ANY MATERIALS OR SERVICES OFFERED BY THIRD PARTIES.
 *
 * COPYRIGHT NOTICE
 * © 2010 Microsoft Corporation.  All rights reserved.
 *
 * MICROSOFT LIMITED PUBLIC LICENSE
 * This license governs use of code marked as "sample" or "example" available on this web site without a license agreement, as provided under the section above titled "NOTICE SPECIFIC TO SOFTWARE AVAILABLE ON THIS WEB SITE." If you use such code (the "software"), you accept this license. If you do not accept the license, do not use the software.
 * 1. Definitions
 *   The terms "reproduce," "reproduction," "derivative works," and "distribution" have the same meaning here as under U.S. copyright law.
 *   A "contribution" is the original software, or any additions or changes to the software.
 *   A "contributor" is any person that distributes its contribution under this license.
 *   "Licensed patents" are a contributor's patent claims that read directly on its contribution.
 * 2. Grant of Rights
 *   (A) Copyright Grant - Subject to the terms of this license, including the license conditions and limitations in section 3, each contributor grants you a non-exclusive, worldwide, royalty-free copyright license to reproduce its contribution, prepare derivative works of its contribution, and distribute its contribution or any derivative works that you create.
 *   (B) Patent Grant - Subject to the terms of this license, including the license conditions and limitations in section 3, each contributor grants you a non-exclusive, worldwide, royalty-free license under its licensed patents to make, have made, use, sell, offer for sale, import, and/or otherwise dispose of its contribution in the software or derivative works of the contribution in the software.
 * 3. Conditions and Limitations
 *   (A) No Trademark License- This license does not grant you rights to use any contributors' name, logo, or trademarks.
 *   (B) If you bring a patent claim against any contributor over patents that you claim are infringed by the software, your patent license from such contributor to the software ends automatically.
 *   (C) If you distribute any portion of the software, you must retain all copyright, patent, trademark, and attribution notices that are present in the software.
 *   (D) If you distribute any portion of the software in source code form, you may do so only under this license by including a complete copy of this license with your distribution.  If you distribute any portion of the software in compiled or object code form, you may only do so under a license that complies with this license.
 *   (E) The software is licensed "as-is." You bear the risk of using it. The contributors give no express warranties, guarantees or conditions.  You may have additional consumer rights under your local laws which this license cannot change. To the extent permitted under your local laws, the contributors exclude the implied warranties of merchantability, fitness for a particular purpose and non-infringement.
 *   (F) Platform Limitation - The licenses granted in sections 2(A) and 2(B) extend only to the software or derivative works that you create that run on a Microsoft Windows operating system product.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/time.h"

#include <limits.h>
#include <time.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
// From http://social.msdn.microsoft.com/Forums/en-US/vcgeneral/thread/430449b3-f6dd-4e18-84de-eebd26a8d668/.
// Notice licensing conditions at the top of this file.
#if defined(_msc_ver) || defined(_msc_extensions)
    #define DELTA_EPOCH_IN_MICROSECS 11644473600000000Ui64
#else
    #define DELTA_EPOCH_IN_MICROSECS 11644473600000000ULL
#endif

/******************************************************************************/
/***   Constants                                                            ***/
/******************************************************************************/
const xme_hal_time_timeHandle_t XME_HAL_TIME_INVALID_TIME_HANDLE =
{
    LONG_MAX, // tv_sec
    LONG_MAX // tv_usec
};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
// From http://social.msdn.microsoft.com/Forums/en-US/vcgeneral/thread/430449b3-f6dd-4e18-84de-eebd26a8d668/.
// Notice licensing conditions at the top of this file.
static int
_gettimeofday
(
    struct timeval* tv,
    struct timezone* tz
)
{
    FILETIME ft;
    unsigned __int64 tmpres = 0;

    XME_ASSERT(NULL != tv);
    XME_ASSERT(NULL == tz); // not supported

    GetSystemTimeAsFileTime(&ft);

    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;

    // Converting file time to unix epoch
    tmpres /= 10; // Convert to microseconds
    tmpres -= DELTA_EPOCH_IN_MICROSECS;
    tv->tv_sec = (long)(tmpres / 1000000UL);
    tv->tv_usec = (long)(tmpres % 1000000UL);

    return 0;
}

#if 0
// Adapted from the discussion at
// http://www.qtcentre.org/threads/1606-Current-Time-with-microsecond-precision-on-windows
static int
_gettimeofday
(
    struct timeval *tv,
    struct timezone *tz
)
{
    time_t rawtime;
    LARGE_INTEGER tickPerSecond;
    LARGE_INTEGER tick;

    XME_ASSERT(NULL != tv);
    XME_ASSERT(NULL == tz);

    time(&rawtime);
    tv->tv_sec = (long)rawtime;

    QueryPerformanceFrequency(&tickPerSecond);
    QueryPerformanceCounter(&tick);
    tv->tv_usec = (tick.QuadPart % tickPerSecond.QuadPart);

    return 0;
}
#endif

bool
xme_hal_time_isValidTimeHandle
(
    xme_hal_time_timeHandle_t timeHandle
)
{
    return (XME_HAL_TIME_INVALID_TIME_HANDLE.tv_sec != timeHandle.tv_sec ||
        XME_HAL_TIME_INVALID_TIME_HANDLE.tv_usec != timeHandle.tv_usec);
}

xme_hal_time_timeHandle_t
xme_hal_time_getCurrentTime(void)
{
    struct timeval tv;
    _gettimeofday(&tv, NULL);
    return tv;
}

int8_t
xme_hal_time_compareTime
(
    xme_hal_time_timeHandle_t time1,
    xme_hal_time_timeHandle_t time2
)
{
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(time1)), -1);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(time2)), -1);

    if (time1.tv_sec == time2.tv_sec)
    {
        if (time1.tv_usec < time2.tv_usec)
        {
            return -1;
        }
        else if (time1.tv_usec > time2.tv_usec)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else if (time1.tv_sec < time2.tv_sec)
    {
        return -1;
    }
    else // if (time1.tv_sec > time2.tv_sec)
    {
        return 1;
    }
}

// Inspired by tv_util of libgpl:
// http://www.geonius.com/software/libgpl/tv_util.html
xme_hal_time_timeHandle_t
xme_hal_time_offsetTime
(
    xme_hal_time_timeHandle_t time,
    xme_hal_time_offsetOperation_t operation,
    xme_hal_time_timeInterval_t offset
)
{
    uint32_t intervalSeconds;
    long intervalMicroseconds;

    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(time)), XME_HAL_TIME_INVALID_TIME_HANDLE);
    XME_ASSERT_RVAL(time.tv_usec >= 0L, time);
    XME_ASSERT_RVAL(time.tv_usec < 1000000L, time);

    intervalSeconds = xme_hal_time_timeIntervalInSeconds(offset);
    intervalMicroseconds = (long)(xme_hal_time_timeIntervalInMicroseconds(offset) % 1000000ULL);

    switch (operation)
    {
        case XME_HAL_TIME_OFFSET_OPERATION_ADD:
            time.tv_sec += intervalSeconds;
            time.tv_usec += intervalMicroseconds; // this is guaranteed to not overflow
            if (time.tv_usec >= 1000000L)
            {
                time.tv_sec++;
                time.tv_usec -= 1000000L;
            }
            break;

        case XME_HAL_TIME_OFFSET_OPERATION_SUBTRACT:
            if ((time.tv_sec < (long)intervalSeconds) || ((time.tv_sec == (long)intervalSeconds) && (time.tv_usec <= intervalMicroseconds)))
            {
                time.tv_sec = time.tv_usec = 0;
            }
            else
            {
                time.tv_sec -= intervalSeconds;

                if (time.tv_usec < intervalMicroseconds)
                {
                    time.tv_usec += 1000000L - intervalMicroseconds;
                    time.tv_sec--;
                }
                else
                {
                    time.tv_usec -= intervalMicroseconds;
                }
            }
            break;

        default:
            XME_ASSERT_RVAL
            (
                XME_HAL_TIME_OFFSET_OPERATION_ADD == operation ||
                XME_HAL_TIME_OFFSET_OPERATION_SUBTRACT == operation,
                time
            );
    }

    return time;
}

xme_hal_time_timeInterval_t
xme_hal_time_getTimeInterval
(
    xme_hal_time_timeHandle_t* startTime,
    bool reset
)
{
    xme_hal_time_timeInterval_t interval;
    xme_hal_time_timeHandle_t now;

    XME_ASSERT(NULL != startTime);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(*startTime)), 0);

    now = xme_hal_time_getCurrentTime();
    interval = xme_hal_time_getTimeIntervalBetween(*startTime, now);

    if (reset)
    {
        *startTime = now;
    }

    return interval;
}

xme_hal_time_timeInterval_t
xme_hal_time_getTimeIntervalBetween
(
    xme_hal_time_timeHandle_t startTime,
    xme_hal_time_timeHandle_t stopTime
)
{
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(startTime)), 0);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(stopTime)), 0);

    return ((xme_hal_time_timeInterval_t) (stopTime.tv_sec - startTime.tv_sec)) * 1000000000ULL + (stopTime.tv_usec - startTime.tv_usec) * 1000ULL;
}

void
xme_hal_time_destroyTimeHandle
(
    xme_hal_time_timeHandle_t timeHandle
)
{
    // Nothing to do
    XME_UNUSED_PARAMETER(timeHandle);
}

xme_hal_time_timeHandle_t
xme_hal_time_handleFromAbsoluteTime
(
    struct tm* absoluteTime
)
{
    struct timeval result;
    time_t time;

    XME_ASSERT_RVAL(NULL != absoluteTime, XME_HAL_TIME_INVALID_TIME_HANDLE);

    time = mktime(absoluteTime);
    
    XME_CHECK(-1 != time, XME_HAL_TIME_INVALID_TIME_HANDLE);

    result.tv_sec = (long)time;
    result.tv_usec = 0;

    return result;
}

xme_status_t
xme_hal_time_absoluteTimeFromHandle
(
    xme_hal_time_timeHandle_t timeHandle,
    struct tm* absoluteTime
)
{
    struct timeval* tv = (struct timeval*) &timeHandle;
    time_t time = (time_t) tv->tv_sec;
    errno_t status;

    XME_ASSERT(NULL != absoluteTime);

    status = localtime_s(absoluteTime, &time);
    XME_CHECK(0 == status, XME_STATUS_INVALID_PARAMETER);

    return XME_STATUS_SUCCESS;
}

size_t
xme_hal_time_formatTime
(
    char* buffer,
    size_t sizeInBytes,
    const char* format,
    xme_hal_time_timeHandle_t timeHandle
)
{
    struct timeval* tv;
    time_t time;
    struct tm tmv;
    errno_t status;

    tv = (struct timeval*) &timeHandle;
    time = tv->tv_sec;

    status = localtime_s(&tmv, &time);
    XME_ASSERT_RVAL(0 == status, 0);

    return strftime(buffer, sizeInBytes, format, &tmv);
}
