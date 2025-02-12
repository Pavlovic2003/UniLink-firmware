/*
 * SCPI_lib.h
 *
 *  Created on: Apr 8, 2024
 *      Author: kuban
 */

#ifndef INC_UNILINK_SCPI_H_
#define INC_UNILINK_SCPI_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

enum functionTypes { LED_F = 0, VAR_F = 1 };

enum paramTypes { OFF_P = 0, ON_P = 1, EVAL_P = 2, INT_P = 3, OTHER_P = 4 };

enum subWordTypes { function = 0, params = 1 };

typedef struct subword
{
	enum subWordTypes type;
	enum functionTypes function;
	enum paramTypes paramType;
	int integerParam;
	char* otherParam;
}Subword;

typedef struct word
{
	int address;
	struct subword* subwords;
	int subwordsCount;
}Word;

typedef struct function
{
	char* name;
	void (*run)(struct subword*, int);
}Function;


void ReformatString(char* chararr, int arrMaxSize);

struct subword generateSubwordn(char* subcommand, int length);

struct word generateWordDirect(char* command);

void executeWord(struct word word);

void addFunction(char *name, void (*func)(struct subword*, int));

#endif /* INC_SCPI_LIB_H_ */
