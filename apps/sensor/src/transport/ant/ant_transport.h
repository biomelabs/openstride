#ifndef ANT_TRANSPORT_H_
#define ANT_TRANSPORT_H_

#include "sdm/sdm_data.h"

int ant_transport_init(void);
int ant_transport_post(const sdm_data_t *data);

#endif /* ANT_TRANSPORT_H_ */
