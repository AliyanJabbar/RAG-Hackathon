## 1. Environment Setup
- [X] Install dependencies: `cohere`, `qdrant-client`, `python-dotenv`.
- [X] Add `.env` file with required API keys and collection name.

## 2. Client Initialization
- [X] Initialize Cohere client with `COHERE_API_KEY`.
- [X] Initialize Qdrant client with `QDRANT_URL` and `QDRANT_API_KEY`.

## 3. Embedding Function
- [X] Implement `get_embedding(text)` function.
- [X] Ensure embedding output size matches Qdrant collection.

## 4. Data Retrieval Function
- [X] Implement `retrieve_data(query)` function.
- [X] Query Qdrant collection with the embedding.
- [X] Extract `text` payload from top 5 results.

## 5. Agent Tool Integration
- [X] Decorate `retrieve_data` with `@function_tool`.
- [X] Ensure function is callable by agents.

## 6. Testing & Validation
- [X] Test `retrieve_data` with sample queries.
- [X] Verify returned results are semantically relevant.
