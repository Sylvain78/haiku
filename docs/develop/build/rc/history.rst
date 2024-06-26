History
#######

This is the change history of the rc resource compiler and librdef.

Version 1.1 (December 22, 2003)
===============================

You can now import the contents of external files (such as pictures) into array resources or fields. This does away with the need to convert (large) binary files into text form.

You can now give type fields a fixed size, which can be handy for certain kinds of string or raw data resources (for example, app_version).

You can now use built-in symbolic constants such as B_SINGLE_LAUNCH for setting the app_flags and
app_version resources. There is no facility for defining your own symbols yet. (In case you were
wondering, no, you cannot use the enum statement for that.)

Added limited support for numerical expressions. You can now do simple math like `(1 + 2) * 3`.
Only a few operators are supported, you can only use integer operands, and the results are always
cast to 32-bit integer as well. This should be more than enough for now; expressions can be made
more powerful in the future should the need arise. You'll use this feature mostly for OR'ing and
AND'ing symbolic constants (see above) together to make your scripts more readable.

Line number counter was not reset properly when compiling multiple rdef files in one go.

Bumped the API version number of librdef.so to 2.

Version 1.0 (February 16, 2003)
===============================

Initial release
