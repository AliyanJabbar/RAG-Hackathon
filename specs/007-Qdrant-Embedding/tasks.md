## 1. Environment Setup
- [X] Install dependencies: `qdrant-client`, `cohere`, `trafilatura`, `requests`, `python-dotenv`.
- [X] Add `.env` file with required API keys and collection name.

## 2. Sitemap Handling
- [X] Implement `get_all_urls()` to parse sitemap XML.
- [X] Log extracted URLs.

## 3. Text Extraction
- [X] Implement `extract_text_from_url(url)`.
- [X] Handle pages with missing text.

## 4. Chunking
- [X] Implement `chunk_text(text)` with sentence-aware splitting.

## 5. Embedding Generation
- [X] Implement `embed(text)` function using Cohere.

## 6. Qdrant Collection Management
- [X] Implement `create_collection()` to check/create collection.

## 7. Data Storage in Qdrant
- [X] Implement `save_chunk_to_qdrant(chunk, chunk_id, url)`.

## 8. Pipeline Execution
- [X] Implement `ingest_book()` to orchestrate all steps.
- [X] Increment `global_id` for each chunk.
- [X] Print ingestion summary.

## 9. Testing & Validation
- [X] Test pipeline on a small sitemap.
- [X] Verify chunks and embeddings in Qdrant.
