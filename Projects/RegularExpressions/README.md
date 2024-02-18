# Regular Expressions Project

## Introduction

This document serves as a detailed overview of my completion process for the Regular Expressions project, undertaken on September 27th, 2023. The project was structured into four key segments, each aimed at addressing specific functionalities required for the application, including file reading, Regular Expression (RegEx) matching, main program execution, and extensive testing.

## Project Breakdown

### 1. File Reading Implementation

The first step in my project was to implement the `read_file()` function within `functions.c`. This function was designed to accurately process the input file, which starts with an integer indicating the total number of words (`size`), followed by the words listed on separate lines. I created a loop that iterates `size` times, each time using `fscanf()` to read a word and store it in an array named `words`. Error handling was integrated to ensure the function would terminate and return an error flag in case of file reading issues.

### 2. Recursive RegEx Matching

Following the file reading, I focused on implementing RegEx matching through a recursive strategy in the `match()` function, also located in `functions.c`. My approach dissected both the RegEx and the word into segments, sequentially addressing one set of RegEx characters at a time and passing the remainder into recursive function calls. This method allowed me to handle various patterns, including empty strings, special characters (tilde, asterisk, question mark), and direct character matches, through a mix of greedy checks and strategic backtracking.

### 3. Main Program Execution

My next task was the execution logic in `hw3.c`, where I managed user interactions and the application's overall flow. I implemented input validation to gracefully handle invalid file or RegEx restrictions, and functionality to open the file, read the word count, receive a RegEx expression from the user, and apply the `match()` function to identify and display any matches. In cases where no matches were found, I ensured users were appropriately notified.

### 4. Testing and Validation

I concluded my project work with a focus on testing and validation, executed through `test_functions.c`. After reviewing the given test cases, I ideated additional scenarios to rigorously test the `read_file()` and `match()` functions against a broad spectrum of RegEx expressions, aiming to confirm the program's robustness and precision across diverse inputs.

## Development Strategy

My approach to the project was systematic and incremental, with a keen focus on tackling each segment thoroughly and efficiently. I meticulously considered the project's requirements, potential edge cases, and the overarching goal of developing a resilient and effective solution. This structured approach allowed me to navigate through each challenge methodically and enhance my proficiency in RegEx applications.

## Conclusion

This README.md encapsulates the journey I embarked on to complete the Regular Expressions project. Through strategic planning, meticulous implementation, and comprehensive testing, I have crafted a solution that not only fulfills the project's stipulations but also demonstrates my capability to resolve intricate programming challenges with thoughtful analysis and technical acumen.

