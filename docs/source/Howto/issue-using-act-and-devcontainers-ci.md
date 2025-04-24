# Issue using `act` and `devcontainers/ci`

The problem is that the **mounted workspace** in the
`devcontainers/ci` docker container is **from the host file system**.

`act` is creating its own docker container with the workspace copied
into its container’s file system. `devcontainers/ci` docker ntainer is
not created inside `act`’s container (no docker-in-docker), but
besides it. Therefore `devcontainers/ci`’s workspace mount is t
pointing to `act`’s workspace copy, but to the original workspace on
the host file system.

This is why the devcontainer cannot start. Path
`/var/run/act/workflow/` exists in `act`’s container but not on
your host system:

```
| [2023-11-17T09:14:55.111Z] Start: Run: docker run
...
--mount type=bind,src=/var/run/act/workflow/outputcmd.txt,dst=/mnt/github/output
...
| [2023-11-17T09:14:55.386Z] docker: Error response from daemon: invalid mount config for type "bind": bind source path does not exist: /r/run/act/workflow/outputcmd.txt.
```

This can be worked around by creating the files under
`/var/run/act/workflow/` on the host filesystem, so the devcontainer
can start, BUT the `devcontainers/ci` step will work on a different
file system from all the other steps in the job, running it with `act`
will be ssibly pointless.

*Not sure how \`\`act\`\` should handle these kind of scenarios, but maybe
if instead of copying the workspace into its container it could create a
temporary copy of the workspace and the necessary files (like
\`\`/var/run/act/workflow/\`\`) on the host file system. Then could bind
mount this copy into all of the containers created for the given job run
and all the steps of the job would work on the same copied workspace on
the host file system.*

## Docker in Docker (DinD) and (Docker outside of Docker DooD)

<https://shisho.dev/blog/posts/docker-in-docker/>

### Source

<https://github.com/nektos/act/issues/2095#issuecomment-2015817810>
