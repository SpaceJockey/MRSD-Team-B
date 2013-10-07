/* ComTypes.cpp - Communication classes implementation
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>
#import "Comtypes.h"

//Check a packet to verify no transmission errors
bool is_valid(Packet p){
	//Check headers and footers
	if ((p.data.header != 0xDEAD) || (p.data.footer != 0xBEEF)) return false;
	
	//calculate checksum
	uint16_t checksum = 0;
	for(int i = 0; i < 8; i++) checksum += p.data.channel[i];
	if(checksum != p.data.checksum) return false;

	//Looks ok!
	return true;
}