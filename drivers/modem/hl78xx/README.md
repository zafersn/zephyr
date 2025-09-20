HL78XX modem driver — chat helper notes
=====================================

This small file documents the chat/URC refactor used by the HL78XX driver.

- The macro-generated `MODEM_CHAT_MATCH_*` and `MODEM_CHAT_SCRIPT_*` objects
  have been centralized in `hl78xx_chat.c` to avoid other translation units
  taking their addresses or using `ARRAY_SIZE()` at file scope.

- Accessors in `hl78xx_chat.h` provide runtime access to the objects:
  - `hl78xx_get_*()` and `hl78xx_get_*_size()` functions return pointer/size
    information for match arrays.
  - `hl78xx_run_*_script[_async]()` functions run scripts via the modem
    chat API at runtime.

- Device instance configuration sets `.init_chat_script` and
  `.periodic_chat_script` to NULL; scripts are invoked at runtime using the
  wrapper runners. This prevents cross-TU constant-expression issues.

Restore / rollback
------------------
If you need to revert the change, move the MODEM_CHAT_* definitions back into
the driver TUs and remove the wrapper functions; ensure any `ARRAY_SIZE()`
or address-taking usages are adjusted accordingly.
