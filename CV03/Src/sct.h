/*
 * sct.h
 *
 *  Created on: 16. 10. 2020
 *      Author: Pavel
 */

#ifndef SCT_H_
#define SCT_H_

	#define sct_nla(x) do { if (x) GPIOB->BSRR = (1 << 5); else GPIOB->BRR = (1 << 5); } while (0)


#endif /* SCT_H_ */
