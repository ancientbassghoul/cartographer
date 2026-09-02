# TEST PLAN
## This is a plan made to test if we can have claude code automatically write a plan split to several chunks using Sonnet.
## The plan is supposed to be very simple.
## CHUNK 1 — Write the first file.
Please write a file on disk, named "first_file.txt" containing the character "a".
## CHUNK 2 — Write the second file.
Please write a file on disk, named "second_file.txt" containing the character "b".
## CHUNK 3 — Write the third file.
Please write a file on disk, named "third_file.txt" containing the character "c".
## Last Step
Check that three files named "first_file.txt", "second_file.txt" and "third_file.txt" exist on disk. No need to verify their content.