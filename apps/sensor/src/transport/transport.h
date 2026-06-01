#ifndef TRANSPORT_H_
#define TRANSPORT_H_

#include "sdm/sdm_data.h"

void transport_init(void);
void transport_post(const sdm_data_t *data);

#endif /* TRANSPORT_H_ */