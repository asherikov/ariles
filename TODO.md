Version 2.0
-----------

* Rename methods to indicate that the belong to ariles, e.g., `readConfig()` ->
  `arilesRead()`.

* Investigate possibility of generalization of functionality to enable addition
  of custom operations over Ariles classes, e.g., register/unregister.

* Rename `finalize()` and `setDefaults()` to something like `arilesPreRead()`,
  `arilesPostRead()`. Add the same functionality for writing configuration files.
