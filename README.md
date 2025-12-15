# HMINF

## Usage
The project uses the [uv](https://docs.astral.sh/uv/) package manager. It can be installed using this command on linux:
```shell
curl -LsSf https://astral.sh/uv/install.sh | sh
```
Or using this command on Windows:
```shell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

After uv is installed, run this command for initializing the project:
```shell
uv sync
```

Now, the project can run using:

```shell
uv run python src/main.py       
```

