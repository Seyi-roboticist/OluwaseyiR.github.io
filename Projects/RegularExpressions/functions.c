// Created by Seyi R. Afolayan
// Robotics MSE student at Johns Hopkins University
// Student ID: B9B102
// Credits: Geeks for Geeks, Piazza answers, TAs, and C "How to Program"

/**************************************************BEGIN CODE**********************************************************/
// Included Libraries and Header Files
#include "functions.h"
#include <stdio.h>
#include <string.h>

// PART 1: IMPLEMENTATION OF THE read_file()
int read_file(FILE *fp, char words[][MAX_WORD_SIZE + 1], int size) {
    // TODO: Implement Me!
    // Using a for loop to get the words.
    for (int i = 0; i < size; ++i) {
        // Read word from file (using the file pointer "fp") and copy to words array.
        char word[MAX_WORD_SIZE + 1]; // Initialization of the 1D array
        if (fscanf(fp, "%s\n", word))
            strcpy(words[i], word);
        else
            return 1; // exit code 1.
    }

    return 0; // exit code 0.

}

// PART 2: IMPLEMENTATION OF THE REGEX MATCHING FUNCTION (RECURSIVE BACKTRACKING)
/** Match function to check whether regex matches a word.
 * @param regex, is a null terminated char array of the regex
 * @param word is the null terminated char array of word to match to
 * @param restriction is the restriction size for the tilde operator
 * @return 1 if it is a match, else 0 if it's not a match.
 */
int match(const char *regex, const char *word, int restriction) {
    // If both regex and word are empty, match is found
    if (regex[0] == '\0' && word[0] == '\0') {
        return 1;
        // Else if regex starts with tilde
    } else if (regex[0] == '~') {
        // Loop through each possible restriction length
        for (int i = 0; i <= restriction; i++) {
            // Check recursively if match with restriction length
            if (match(regex + 1, word + i, restriction))
                return 1;
            // If end of word, break
            if (word[i] == '\0')
                break;
        }
        // Else if regex contains [char]*
    } else if (regex[1] == '*') {
        // Check recursively if match with one or more instance of character
        if (regex[0] == word[0] && match(regex, word + 1, restriction))
            return 1;
        // Check recursively if match with no instance of character
        if (match(regex + 2, word, restriction))
            return 1;
    }
        // Else if regex contains [char]?
    else if (regex[1] == '?') {
        // Check recursively if match with one instance of character
        if (regex[0] == word[0] && match(regex + 2, word + 1, restriction))
            return 1;
        // Check recursively if match with no instance of character
        if (match(regex + 2, word, restriction))
            return 1;
    }
        // Else if first character of regex matches with first character of word
    else if (regex[0] == word[0]) {
        // Check if match with rest of regex and rest of word
        return match(regex + 1, word + 1, restriction);
    }
    // Else no match found, return 0
    return 0;
}
