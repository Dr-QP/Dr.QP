# `feat` Commit Reference

Use `feat` for a new feature or new functionality.

Examples extracted from the skill:

```bash
feat(auth): add JWT-based user authentication

- Implement login/logout functionality
- Add token management service
- Include auth guards for protected routes
- Add unit tests for auth service

Closes #42
```

```bash
feat(api): update user API response format

Changed response structure to include metadata
for better pagination and filtering support.

BREAKING CHANGE: User API now returns { data, metadata }
instead of direct array. Update client code accordingly.
```

```bash
feat(auth): implement complete authentication system

- Add JWT token generation and validation
- Implement password hashing with bcrypt
- Create login/logout API endpoints
- Add auth middleware for protected routes
- Include refresh token functionality

Closes #42, #43, #44
```

```text
feat(ui): add responsive navigation menu
feat(api): add user pagination endpoint
```
