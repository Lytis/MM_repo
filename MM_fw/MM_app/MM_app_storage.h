#ifndef MM_APP_STORAGE_H_
#define MM_APP_STORAGE_H_



#define SUB_PACKETS             8
#define TRANSMIT_PACKET_SIZE    SAI_BUFFER_SIZE*SUB_PACKETS + 1

void storage_init(void);
void push_samples(int32_t *, int);


#endif
