#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "JY901.h"

CJY901 ::CJY901()
{
}

void CJY901::FetchData(char *data, int usLength)
{
	char *pData_head = data;
	while (usLength >= 11)
	{
		if (pData_head[0] != 0x55)
		{
			pData_head++;
			continue;
		}
		switch (pData_head[1])
		{
		case 0x50:
			memcpy(&stcTime, &pData_head[2], 8);
			break;
		case 0x51:
			memcpy(&stcAcc, &pData_head[2], 8);
			acc.x = stcAcc.a[0] / 32768.00 * 16 * 9.8;
			acc.y = stcAcc.a[1] / 32768.00 * 16 * 9.8;
			acc.z = stcAcc.a[2] / 32768.00 * 16 * 9.8;
			break;
		case 0x52:
			memcpy(&stcGyro, &pData_head[2], 8);
			gyro.x = stcGyro.w[0] / 32768.00 * 2000 / 180 * 3.1415926;
			gyro.y = stcGyro.w[1] / 32768.00 * 2000 / 180 * 3.1415926;
			gyro.z = stcGyro.w[2] / 32768.00 * 2000 / 180 * 3.1415926;
			break;
		case 0x53:
			memcpy(&stcAngle, &pData_head[2], 8);
			angle.r = stcAngle.Angle[0] / 32768.00 * 3.1415926;
			angle.p = stcAngle.Angle[1] / 32768.00 * 3.1415926;
			angle.y = stcAngle.Angle[2] / 32768.00 * 3.1415926;
			break;
		case 0x54:
			memcpy(&stcMag, &pData_head[2], 8);
			mag.x = stcMag.h[0];
			mag.y = stcMag.h[1];
			mag.z = stcMag.h[2];
			break;
		// case 0x55:	memcpy(&stcDStatus,&chrTemp[2],8);break;
		// case 0x56:	memcpy(&stcPress,&chrTemp[2],8);break;
		// case 0x57:	memcpy(&stcLonLat,&chrTemp[2],8);break;
		// case 0x58:	memcpy(&stcGPSV,&chrTemp[2],8);break;
		case 0x59:
			// printf("\n0x59 Source Data is : { %#.2X %#.2X %#.2X %#.2X %#.2X %#.2X %#.2X %#.2X }", chrTemp[2], chrTemp[3], chrTemp[4], chrTemp[5], chrTemp[6], chrTemp[7], chrTemp[8], chrTemp[9]);
			memcpy(&stcQuat, &pData_head[2], 8);
			// printf("\nSource Data is : { x: %d  y: %d  z: %d  w: %d}", stcQuat.q[1], stcQuat.q[2], stcQuat.q[3], stcQuat.q[0]);
			quat.w = stcQuat.q[0] / 32768.00;
			quat.x = stcQuat.q[1] / 32768.00;
			quat.y = stcQuat.q[2] / 32768.00;
			quat.z = stcQuat.q[3] / 32768.00;
			break;
		}
		usLength -= 11;
		pData_head += 11;
	}
}