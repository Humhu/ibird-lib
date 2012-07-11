// Taken without permission from Microchip Forums, Jack Peacock
// Modified by Humphrey Hu

unsigned char sqrtI(unsigned int sqrtArg) {
    
    unsigned char answer, x;
    unsigned int temp;

    if ( sqrtArg == 0 ) return 0; // undefined result
    if ( sqrtArg == 1 ) return 1; // identity
    answer = 0;                     // integer square root
    for( x=0x80; x>0; x=x>>1 ) {                        
        answer |= x;             // possible bit in root
        temp = answer*answer;
        if (temp == sqrtArg) break; // exact, found it
        if (temp > sqrtArg) answer ^= x; // too large, reverse bit
    }
    return answer; // approximate root
    
}
