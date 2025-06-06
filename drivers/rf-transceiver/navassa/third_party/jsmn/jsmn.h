#ifndef __JSMN_H_
#define __JSMN_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief The `jsmntype_t` is an enumeration that defines the basic types of
 * JSON data structures that can be parsed by the JSMN (JSON Minimalist
 * Parser). It includes types for JSON objects, arrays, strings, and
 * primitive values like numbers, booleans, and nulls, as well as an
 * undefined type for error handling or uninitialized states.
 *
 * @param JSMN_UNDEFINED Represents an undefined JSON type.
 * @param JSMN_OBJECT Represents a JSON object type.
 * @param JSMN_ARRAY Represents a JSON array type.
 * @param JSMN_STRING Represents a JSON string type.
 * @param JSMN_PRIMITIVE Represents a JSON primitive type, such as number,
 * boolean, or null.
 ******************************************************************************/
typedef enum {
	JSMN_UNDEFINED = 0,
	JSMN_OBJECT = 1,
	JSMN_ARRAY = 2,
	JSMN_STRING = 3,
	JSMN_PRIMITIVE = 4
} jsmntype_t;

/***************************************************************************//**
 * @brief The `jsmnerr` enumeration defines error codes used by the JSMN JSON
 * parser to indicate specific parsing errors, such as insufficient
 * tokens, invalid characters, or incomplete JSON data.
 *
 * @param JSMN_ERROR_NOMEM Indicates that not enough tokens were provided for
 * parsing.
 * @param JSMN_ERROR_INVAL Indicates an invalid character was found inside a
 * JSON string.
 * @param JSMN_ERROR_PART Indicates that the JSON string is incomplete and more
 * bytes are expected.
 ******************************************************************************/
enum jsmnerr {
	/* Not enough tokens were provided */
	JSMN_ERROR_NOMEM = -1,
	/* Invalid character inside JSON string */
	JSMN_ERROR_INVAL = -2,
	/* The string is not a full JSON packet, more bytes expected */
	JSMN_ERROR_PART = -3
};

/***************************************************************************//**
 * @brief The `jsmntok_t` structure is used to represent a token in a JSON data
 * string, capturing the type of JSON element, its position within the
 * string, and its hierarchical structure if parent links are enabled. It
 * is part of a JSON parser that breaks down a JSON string into
 * manageable tokens for further processing.
 *
 * @param type Specifies the type of JSON data (object, array, string, or
 * primitive).
 * @param start Indicates the starting position of the token in the JSON data
 * string.
 * @param end Indicates the ending position of the token in the JSON data
 * string.
 * @param size Represents the number of child tokens for arrays and objects.
 * @param parent Stores the index of the parent token, if JSMN_PARENT_LINKS is
 * defined.
 ******************************************************************************/
typedef struct {
	jsmntype_t type;
	int start;
	int end;
	int size;
#ifdef JSMN_PARENT_LINKS
	int parent;
#endif
} jsmntok_t;

/***************************************************************************//**
 * @brief The `jsmn_parser` structure is used to maintain the state of the JSON
 * parsing process. It keeps track of the current position in the JSON
 * string (`pos`), the index of the next token to be allocated
 * (`toknext`), and the index of the superior token node (`toksuper`),
 * which helps in managing hierarchical JSON structures such as objects
 * and arrays.
 *
 * @param pos Offset in the JSON string.
 * @param toknext Next token to allocate.
 * @param toksuper Superior token node, e.g., parent object or array.
 ******************************************************************************/
typedef struct {
	unsigned int pos; /* offset in the JSON string */
	unsigned int toknext; /* next token to allocate */
	int toksuper; /* superior token node, e.g parent object or array */
} jsmn_parser;

/***************************************************************************//**
 * @brief Use this function to prepare a `jsmn_parser` structure for parsing a
 * JSON string. It must be called before any parsing operations to ensure
 * the parser is in a known initial state. This function sets the
 * internal state of the parser to the beginning of a JSON string, ready
 * to allocate tokens and track parsing progress. It does not allocate
 * memory or perform any operations other than resetting the parser's
 * state.
 *
 * @param parser A pointer to a `jsmn_parser` structure that will be
 * initialized. The caller must ensure this pointer is valid and
 * points to a writable memory location. The function does not
 * check for null pointers, so passing a null pointer will result
 * in undefined behavior.
 * @return None
 ******************************************************************************/
void jsmn_init(jsmn_parser *parser);

/***************************************************************************//**
 * @brief This function parses a JSON-formatted string and populates an array of
 * tokens, each representing a JSON object, array, string, or primitive.
 * It should be called after initializing a `jsmn_parser` structure using
 * `jsmn_init`. The function processes the input JSON string up to the
 * specified length and fills the provided token array with parsed
 * elements. If the token array is too small to hold all parsed tokens,
 * the function returns an error. The function can handle both strict and
 * non-strict JSON parsing, depending on the compilation flags. It is
 * important to ensure that the `tokens` array is large enough to
 * accommodate the expected number of JSON elements to avoid memory
 * errors.
 *
 * @param parser A pointer to an initialized `jsmn_parser` structure. The parser
 * must be initialized using `jsmn_init` before calling this
 * function. The caller retains ownership.
 * @param js A pointer to the JSON data string to be parsed. The string must be
 * null-terminated, and the caller retains ownership.
 * @param len The length of the JSON data string to be parsed. It must be a non-
 * negative value and should not exceed the actual length of the
 * string.
 * @param tokens A pointer to an array of `jsmntok_t` structures where parsed
 * tokens will be stored. It can be `NULL` if only the count of
 * tokens is needed. The caller retains ownership.
 * @param num_tokens The number of tokens available in the `tokens` array. It
 * must be a non-negative value. If the number of tokens is
 * insufficient, the function returns an error.
 * @return Returns the number of tokens parsed on success. On error, returns a
 * negative value indicating the type of error, such as
 * `JSMN_ERROR_NOMEM`, `JSMN_ERROR_INVAL`, or `JSMN_ERROR_PART`.
 ******************************************************************************/
int jsmn_parse(jsmn_parser *parser, const char *js, size_t len,
		jsmntok_t *tokens, unsigned int num_tokens);

#ifdef __cplusplus
}
#endif

#endif /* __JSMN_H_ */
